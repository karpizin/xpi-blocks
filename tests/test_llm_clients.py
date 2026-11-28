"""LLM clients: tool calling with fake transports, no network and no API keys.

Covered: Gemini API (Gemini and Gemma 4), Ollama (Qwen 3.x, Gemma 4), llama.cpp
llama-server and other OpenAI-compatible servers, Qwen on DashScope, OpenRouter.

Gemini runs through the real google-genai SDK; only its HTTP layer is replaced, so
the request it would send (function declarations) and the parsing of the reply
(function calls in parts[].function_call) are the SDK's own.
"""
import json

import pytest
import requests

from xpi_llm.llm_clients import (DEFAULT_GEMINI_MODEL, GeminiClient, OllamaClient,
                                 OpenRouterClient, llm_client_factory, openai_tools_to_gemini,
                                 parse_think)

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


def test_gemma4_on_gemini_api(gemini):
    """Gemma 4 is served by the Gemini API too: same client, just the model name."""
    client, sent, replies = gemini
    client.model = 'gemma-4-26b-a4b-it'
    replies.append(_function_call_reply("set_relay_state", {"relay_id": 2.0, "state": False}))
    assert client.generate("relay 2 off", tools=[RELAY_TOOL])[1]['name'] == 'set_relay_state'
    assert sent[0][1] == 'models/gemma-4-26b-a4b-it:generateContent'


# ---------------------------------------------------------------- HTTP clients (fake requests.post)
class FakeResponse:
    def __init__(self, body, status=200):
        self.body, self.status_code = body, status
        self.text = body if isinstance(body, str) else json.dumps(body)

    def raise_for_status(self):
        if self.status_code >= 400:
            raise requests.exceptions.HTTPError(f'{self.status_code}: {self.text}')

    def json(self):
        if isinstance(self.body, str):
            raise ValueError('not JSON')
        return self.body


@pytest.fixture
def post(monkeypatch):
    calls, replies = [], []

    def fake_post(url, json=None, headers=None, timeout=None):
        calls.append({'url': url, 'json': json, 'headers': headers or {}})
        reply = replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply
    monkeypatch.setattr(requests, 'post', fake_post)
    for var in ('GEMINI_API_KEY', 'DASHSCOPE_API_KEY', 'OPENROUTER_API_KEY', 'OPENAI_API_KEY', 'LLAMA_API_KEY'):
        monkeypatch.delenv(var, raising=False)
    return calls, replies


def ollama_reply(content='', tool_calls=None, thinking=None):
    message = {'role': 'assistant', 'content': content}
    if tool_calls is not None:
        message['tool_calls'] = tool_calls
    if thinking:
        message['thinking'] = thinking
    return FakeResponse({'model': 'm', 'message': message, 'done': True})


def openai_reply(content=None, tool_calls=None):
    message = {'role': 'assistant', 'content': content}
    if tool_calls is not None:
        message['tool_calls'] = tool_calls
    return FakeResponse({'choices': [{'message': message, 'finish_reason': 'stop'}]})


# --- Ollama (/api/chat, native tool calling: Qwen 3.x, Gemma 4)
def test_ollama_native_tool_call(post):
    calls, replies = post
    replies.append(ollama_reply(tool_calls=[{'function': {'name': 'set_relay_state',
                                                          'arguments': {'relay_id': 5, 'state': True}}}]))
    client = OllamaClient(model='gemma4:e4b')
    text, call = client.generate("Turn on relay 5", tools=[RELAY_TOOL], temperature=0.1, max_tokens=64)
    assert call == {'name': 'set_relay_state', 'arguments': {'relay_id': 5, 'state': True}}
    req = calls[0]
    assert req['url'] == 'http://localhost:11434/api/chat'
    assert req['json']['model'] == 'gemma4:e4b' and req['json']['tools'] == [RELAY_TOOL]
    assert req['json']['messages'] == [{'role': 'user', 'content': 'Turn on relay 5'}]
    assert req['json']['options'] == {'temperature': 0.1, 'num_predict': 64}
    assert 'think' not in req['json']


def test_ollama_tool_arguments_as_json_string(post):
    calls, replies = post
    replies.append(ollama_reply(tool_calls=[{'function': {'name': 'set_relay_state',
                                                          'arguments': '{"relay_id": 5, "state": true}'}}]))
    assert OllamaClient().generate("x", tools=[RELAY_TOOL])[1]['arguments'] == {'relay_id': 5, 'state': True}


def test_ollama_default_model_is_qwen():
    assert OllamaClient().model == 'qwen3.5:4b'


def test_ollama_model_without_tools_falls_back_to_prompt(post):
    """HTTP 400 "does not support tools": ask for a JSON tool call in the prompt instead."""
    calls, replies = post
    replies.append(FakeResponse('{"error":"registry.ollama.ai/library/llava:latest does not support tools"}', 400))
    replies.append(ollama_reply(content='```json\n{"tool_call": {"name": "set_relay_state", '
                                        '"arguments": {"relay_id": "5", "state": "true"}}}\n```'))
    client = OllamaClient(model='llava')
    text, call = client.generate("Turn on relay 5", tools=[RELAY_TOOL])
    assert call == {'name': 'set_relay_state', 'arguments': {'relay_id': '5', 'state': 'true'}}
    assert 'tools' in calls[0]['json'] and 'tools' not in calls[1]['json']
    assert 'set_relay_state' in calls[1]['json']['messages'][0]['content']
    # remembered: the next call goes straight to the prompt fallback
    replies.append(ollama_reply(content='OK'))
    client.generate("again", tools=[RELAY_TOOL])
    assert len(calls) == 3 and 'tools' not in calls[2]['json']


def test_ollama_think_parameter_and_thinking_never_returned(post):
    calls, replies = post
    replies.append(ollama_reply(content='<think>the user wants...</think>Relay 5 is on.', thinking='hidden'))
    text, call = OllamaClient(think=False).generate("x")
    assert text == 'Relay 5 is on.'
    assert calls[0]['json']['think'] is False


def test_ollama_image(post):
    calls, replies = post
    replies.append(ollama_reply(content='a robot'))
    OllamaClient().generate("What is this?", media_data=b'\xff\xd8\xff')
    assert calls[0]['json']['messages'][0]['images'] == ['/9j/']


@pytest.mark.parametrize('error', [requests.exceptions.ConnectionError('refused'),
                                   requests.exceptions.Timeout('slow'),
                                   requests.exceptions.HTTPError('500')])
def test_ollama_errors_return_text_and_empty_tool(post, error):
    calls, replies = post
    replies.append(error)
    text, call = OllamaClient().generate("Turn on relay 5", tools=[RELAY_TOOL])
    assert text.startswith('Error:') and call == {}


def test_ollama_plain_text(post):
    calls, replies = post
    replies.append(ollama_reply(content="The sky is blue because..."))
    assert OllamaClient().generate("Why?", tools=[RELAY_TOOL]) == ("The sky is blue because...", {})


# --- llama.cpp llama-server / any OpenAI-compatible server
def test_llamacpp_tool_call(post):
    calls, replies = post
    replies.append(openai_reply(tool_calls=[{'id': 'c1', 'type': 'function', 'function': {
        'name': 'set_relay_state', 'arguments': '{"relay_id": 5, "state": true}'}}]))
    client = llm_client_factory('llama.cpp')
    text, call = client.generate("Turn on relay 5", tools=[RELAY_TOOL])
    assert call == {'name': 'set_relay_state', 'arguments': {'relay_id': 5, 'state': True}}
    req = calls[0]
    assert req['url'] == 'http://localhost:8080/v1/chat/completions'
    assert req['json']['tools'] == [RELAY_TOOL] and req['json']['tool_choice'] == 'auto'
    assert 'Authorization' not in req['headers']


def test_llamacpp_on_another_machine_and_reasoning_stripped(post):
    calls, replies = post
    replies.append(openai_reply(content='<think>\nhmm\n</think>\n\nHello!'))
    client = llm_client_factory('llamacpp', host='http://192.168.1.20:8080/v1/', model='qwen3.5-9b')
    assert client.generate("Hi") == ('Hello!', {})
    assert calls[0]['url'] == 'http://192.168.1.20:8080/v1/chat/completions'
    assert calls[0]['json']['model'] == 'qwen3.5-9b'


def test_openai_compatible_image_and_key(post):
    calls, replies = post
    replies.append(openai_reply(content='a cat'))
    client = llm_client_factory('openai', api_key='sk-local', host='http://localhost:1234/v1')
    client.generate("What is this?", media_data=b'\xff\xd8\xff', mime_type='image/jpeg')
    content = calls[0]['json']['messages'][0]['content']
    assert content[1]['image_url']['url'] == 'data:image/jpeg;base64,/9j/'
    assert calls[0]['headers']['Authorization'] == 'Bearer sk-local'


@pytest.mark.parametrize('error', [requests.exceptions.ConnectionError('refused'),
                                   requests.exceptions.Timeout('slow')])
def test_llamacpp_errors_return_tuple(post, error):
    calls, replies = post
    replies.append(error)
    text, call = llm_client_factory('llamacpp').generate("x", tools=[RELAY_TOOL])
    assert text.startswith('Error:') and call == {}


def test_llamacpp_reasoning_off_is_sent_to_the_chat_template(post):
    calls, replies = post
    replies.append(openai_reply(content='ok'))
    llm_client_factory('llamacpp', think=False).generate("x")
    assert calls[0]['json']['chat_template_kwargs'] == {'enable_thinking': False}
    replies.append(openai_reply(content='ok'))
    llm_client_factory('llamacpp').generate("x")
    assert 'chat_template_kwargs' not in calls[1]['json']


def test_llamacpp_out_of_tokens_is_explained(post):
    """Gemma 4 / Qwen with reasoning on can spend all max_tokens thinking (seen with llama-server)."""
    calls, replies = post
    replies.append(FakeResponse({'choices': [{'finish_reason': 'length', 'message': {
        'role': 'assistant', 'content': '', 'reasoning_content': '1. Analyze the request...'}}]}))
    text, call = llm_client_factory('llamacpp').generate("x", tools=[RELAY_TOOL])
    assert 'max_tokens' in text and 'llm_think' in text and call == {}
    replies.append(FakeResponse({'choices': [{'finish_reason': 'length', 'message': {
        'role': 'assistant', 'content': '', 'tool_calls': [{'function': {
            'name': 'set_relay_state', 'arguments': '{"relay_id":5,"state":true'}}]}}]}))
    text, call = llm_client_factory('llamacpp').generate("x", tools=[RELAY_TOOL])
    assert 'cut off' in text and call == {}


# --- Qwen on Alibaba Cloud Model Studio (DashScope)
def test_qwen_cloud(post, monkeypatch):
    calls, replies = post
    monkeypatch.setenv('DASHSCOPE_API_KEY', 'ds-key')
    replies.append(openai_reply(tool_calls=[{'function': {'name': 'set_relay_state',
                                                          'arguments': '{"relay_id": 1, "state": false}'}}]))
    client = llm_client_factory('qwen')
    assert client.generate("relay 1 off", tools=[RELAY_TOOL])[1]['arguments'] == {'relay_id': 1, 'state': False}
    assert calls[0]['url'] == 'https://dashscope-intl.aliyuncs.com/compatible-mode/v1/chat/completions'
    assert calls[0]['headers']['Authorization'] == 'Bearer ds-key'
    assert calls[0]['json']['model'] == 'qwen-plus'


def test_qwen_cloud_reasoning_flag(post):
    calls, replies = post
    replies.append(openai_reply(content='ok'))
    llm_client_factory('qwen', api_key='k', think=False).generate("x")
    assert calls[0]['json']['enable_thinking'] is False
    assert 'chat_template_kwargs' not in calls[0]['json']


def test_qwen_cloud_needs_a_key(post):
    with pytest.raises(ValueError, match='API key'):
        llm_client_factory('dashscope')


# --- OpenRouter
def test_openrouter_bad_tool_arguments_do_not_raise(post):
    calls, replies = post
    replies.append(openai_reply(tool_calls=[{"function": {"name": "set_relay_state", "arguments": "{not json"}}]))
    text, call = OpenRouterClient(api_key='k').generate("x", tools=[RELAY_TOOL])
    assert text.startswith('Error:') and call == {}


def test_openrouter_tool_call(post):
    calls, replies = post
    replies.append(openai_reply(tool_calls=[
        {"function": {"name": "set_relay_state", "arguments": '{"relay_id": 5, "state": true}'}}]))
    text, call = OpenRouterClient(api_key='k').generate("x", tools=[RELAY_TOOL])
    assert call == {'name': 'set_relay_state', 'arguments': {'relay_id': 5, 'state': True}}
    assert calls[0]['url'] == 'https://openrouter.ai/api/v1/chat/completions'
    assert calls[0]['headers']['X-Title'] == 'XPI Blocks LLM'


# ---------------------------------------------------------------- factory
def test_factory_keeps_client_defaults_for_empty_values(post):
    client = llm_client_factory('ollama', model=None, host=None)
    assert client.model == 'qwen3.5:4b' and client.host == 'http://localhost:11434'
    client = llm_client_factory('openrouter', api_key='k', model='')
    assert client.model == 'openai/gpt-3.5-turbo'


def test_factory_unknown_type():
    with pytest.raises(ValueError, match='llamacpp'):
        llm_client_factory('claude-local')


def test_factory_gemini_default_model():
    pytest.importorskip('google.genai')
    assert llm_client_factory('gemini', api_key='k', model=None).model == DEFAULT_GEMINI_MODEL


@pytest.mark.parametrize('value,expected', [('auto', None), ('', None), ('on', True), ('off', False),
                                            ('False', False), (True, True)])
def test_parse_think(value, expected):
    assert parse_think(value) is expected
