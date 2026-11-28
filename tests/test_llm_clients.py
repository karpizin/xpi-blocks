"""LLM clients: tool calling with fake transports, no network and no API keys.

Gemini runs through the real google-genai SDK; only its HTTP layer is replaced, so
the request it would send (function declarations) and the parsing of the reply
(function calls in parts[].function_call) are the SDK's own.
"""
import json

import pytest
import requests

from xpi_llm.llm_clients import (DEFAULT_GEMINI_MODEL, GeminiClient, OllamaClient,
                                 OpenRouterClient, llm_client_factory, openai_tools_to_gemini)

RELAY_TOOL = {
    "type": "function",
    "function": {
        "name": "set_relay_state",
        "description": "Sets a relay.",
        "parameters": {
            "type": "object",
            "properties": {
                "relay_id": {"type": "integer", "minimum": 0, "maximum": 15},
                "state": {"type": "boolean"},
            },
            "required": ["relay_id", "state"],
        },
    },
}
NO_ARGS_TOOL = {"type": "function", "function": {"name": "ping", "description": "No arguments.",
                                                  "parameters": {"type": "object", "properties": {}}}}


# ---------------------------------------------------------------- Gemini (google-genai)
@pytest.fixture
def gemini():
    genai = pytest.importorskip('google.genai')
    from google.genai import types
    sent, replies = [], []

    def fake_request(http_method, path, request_dict, http_options=None):
        sent.append((http_method, path, request_dict))
        reply = replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return types.HttpResponse(headers={}, body=json.dumps(reply))

    client = genai.Client(api_key='test-key')
    client._api_client.request = fake_request
    return GeminiClient(api_key=None, client=client), sent, replies


def _function_call_reply(name, args):
    return {"candidates": [{"content": {"role": "model",
                                        "parts": [{"functionCall": {"name": name, "args": args}}]}}]}


def test_gemini_sends_function_declarations_not_openai_wrapper(gemini):
    client, sent, replies = gemini
    replies.append(_function_call_reply("set_relay_state", {"relay_id": 5.0, "state": True}))
    client.generate("Turn on relay 5", tools=[RELAY_TOOL], temperature=0.1, max_tokens=50)
    method, path, body = sent[0]
    assert path == f'models/{DEFAULT_GEMINI_MODEL}:generateContent'
    decl = body['tools'][0]['functionDeclarations'][0]
    assert decl['name'] == 'set_relay_state'
    assert set(decl['parameters']['properties']) == {'relay_id', 'state'}
    assert body['generationConfig'] == {'temperature': 0.1, 'maxOutputTokens': 50}


def test_gemini_parses_function_call_from_parts(gemini):
    client, sent, replies = gemini
    replies.append(_function_call_reply("set_relay_state", {"relay_id": 5.0, "state": True}))
    text, call = client.generate("Turn on relay 5", tools=[RELAY_TOOL])
    assert text == ''
    assert call == {'name': 'set_relay_state', 'arguments': {'relay_id': 5.0, 'state': True}}


def test_gemini_text_reply(gemini):
    client, sent, replies = gemini
    replies.append({"candidates": [{"content": {"role": "model", "parts": [{"text": "Hello"}]}}]})
    assert client.generate("Hi") == ('Hello', {})
    assert 'tools' not in sent[0][2]


def test_gemini_tool_without_arguments(gemini):
    client, sent, replies = gemini
    replies.append(_function_call_reply("ping", {}))
    assert client.generate("ping", tools=[NO_ARGS_TOOL]) == ('', {'name': 'ping', 'arguments': {}})
    assert 'parameters' not in sent[0][2]['tools'][0]['functionDeclarations'][0]


def test_gemini_image_is_sent_inline(gemini):
    client, sent, replies = gemini
    replies.append({"candidates": [{"content": {"role": "model", "parts": [{"text": "a cat"}]}}]})
    client.generate("What is this?", media_data=b'\xff\xd8\xff', mime_type='image/jpeg')
    parts = sent[0][2]['contents'][0]['parts']
    assert parts[0] == {'text': 'What is this?'}
    assert parts[1]['inlineData']['mime_type'] == 'image/jpeg'


def test_gemini_blocked_prompt(gemini):
    client, sent, replies = gemini
    replies.append({"candidates": [], "promptFeedback": {"blockReason": "SAFETY"}})
    text, call = client.generate("x")
    assert text.startswith('Error:') and 'SAFETY' in text and call == {}


def test_gemini_api_error_returns_tuple(gemini):
    client, sent, replies = gemini
    replies.append(RuntimeError('503 UNAVAILABLE'))
    text, call = client.generate("x", tools=[RELAY_TOOL])
    assert text.startswith('Error: Gemini API error') and call == {}


def test_openai_tools_conversion_rejects_garbage():
    assert openai_tools_to_gemini([RELAY_TOOL])[0]['name'] == 'set_relay_state'
    with pytest.raises(ValueError):
        openai_tools_to_gemini([{"type": "function"}])


# ---------------------------------------------------------------- Ollama / OpenRouter
class FakeResponse:
    def __init__(self, body, status=200):
        self.body, self.status_code = body, status

    def raise_for_status(self):
        if self.status_code >= 400:
            raise requests.exceptions.HTTPError(f'{self.status_code}')

    def json(self):
        if isinstance(self.body, Exception):
            raise self.body
        return self.body


@pytest.fixture
def post(monkeypatch):
    calls, replies = [], []

    def fake_post(url, json=None, headers=None, timeout=None):
        calls.append((url, json))
        reply = replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply
    monkeypatch.setattr(requests, 'post', fake_post)
    return calls, replies


@pytest.mark.parametrize('error', [requests.exceptions.ConnectionError('refused'),
                                   requests.exceptions.Timeout('slow'),
                                   requests.exceptions.HTTPError('500')])
def test_ollama_errors_return_text_and_empty_tool(post, error):
    calls, replies = post
    replies.append(error)
    text, call = OllamaClient().generate("Turn on relay 5", tools=[RELAY_TOOL])
    assert text.startswith('Error:') and call == {}


def test_ollama_tool_call_in_code_fence(post):
    calls, replies = post
    replies.append(FakeResponse({"response": 'Sure!\n```json\n{"tool_call": {"name": "set_relay_state", '
                                             '"arguments": {"relay_id": "5", "state": "true"}}}\n```'}))
    text, call = OllamaClient().generate("Turn on relay 5", tools=[RELAY_TOOL])
    assert call == {'name': 'set_relay_state', 'arguments': {'relay_id': '5', 'state': 'true'}}
    assert 'set_relay_state' in calls[0][1]['prompt']


def test_ollama_plain_text(post):
    calls, replies = post
    replies.append(FakeResponse({"response": "The sky is blue because..."}))
    assert OllamaClient().generate("Why?", tools=[RELAY_TOOL]) == ("The sky is blue because...", {})


def test_openrouter_bad_tool_arguments_do_not_raise(post):
    calls, replies = post
    replies.append(FakeResponse({"choices": [{"message": {"tool_calls": [
        {"function": {"name": "set_relay_state", "arguments": "{not json"}}]}}]}))
    text, call = OpenRouterClient(api_key='k').generate("x", tools=[RELAY_TOOL])
    assert text.startswith('Error:') and call == {}


def test_openrouter_tool_call(post):
    calls, replies = post
    replies.append(FakeResponse({"choices": [{"message": {"content": None, "tool_calls": [
        {"function": {"name": "set_relay_state", "arguments": '{"relay_id": 5, "state": true}'}}]}}]}))
    text, call = OpenRouterClient(api_key='k').generate("x", tools=[RELAY_TOOL])
    assert call == {'name': 'set_relay_state', 'arguments': {'relay_id': 5, 'state': True}}
    assert calls[0][1]['tools'] == [RELAY_TOOL]


# ---------------------------------------------------------------- factory
def test_factory_keeps_client_defaults_for_empty_values():
    client = llm_client_factory('ollama', model=None, host=None)
    assert client.model == 'llava' and client.host == 'http://localhost:11434'
    client = llm_client_factory('openrouter', api_key='k', model='')
    assert client.model == 'openai/gpt-3.5-turbo'


def test_factory_gemini_default_model():
    pytest.importorskip('google.genai')
    assert llm_client_factory('gemini', api_key='k', model=None).model == DEFAULT_GEMINI_MODEL
