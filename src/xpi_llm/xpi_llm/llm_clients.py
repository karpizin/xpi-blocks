import base64
import os
import re
import requests
import json
import logging
from abc import ABC, abstractmethod

# Configure logging for this module
logger = logging.getLogger(__name__)
logger.setLevel(os.environ.get('XPI_LLM_LOGLEVEL', 'INFO').upper())
if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)


class LLMClient(ABC):
    """Abstract base class for LLM clients."""
    @abstractmethod
    def generate(self, prompt: str, tools: list = None, media_data: bytes = None, mime_type: str = "image/jpeg", **kwargs) -> tuple[str, dict]:
        """
        Sends a prompt to the LLM and returns the generated text and/or a tool_call.
        
        Args:
            prompt: The text prompt.
            tools: Optional list of tool definitions.
            media_data: Optional raw bytes of an image or audio file.
            mime_type: MIME type of the media data (e.g. "audio/wav", "image/jpeg").
            
        Returns: (text_response, tool_call_dict)
        """
        pass

    @abstractmethod
    def get_model_name(self) -> str:
        """Returns the name of the LLM model being used."""
        pass

_THINK_RE = re.compile(r"<think>.*?</think>\s*", re.S)


def strip_thinking(text):
    """Drop <think>...</think> blocks that reasoning models (Qwen 3.x, Gemma 4) put in the text."""
    return _THINK_RE.sub("", text or "").strip()


def _parse_arguments(raw):
    """Tool-call arguments: a dict (Ollama) or a JSON string (OpenAI-style APIs). None if invalid."""
    if isinstance(raw, dict):
        return raw
    if raw in (None, ""):
        return {}
    try:
        value = json.loads(raw)
    except (TypeError, ValueError):
        return None
    return value if isinstance(value, dict) else None


class OpenAICompatibleClient(LLMClient):
    """Any server with the OpenAI chat-completions API and tool calling.

    llama.cpp's llama-server (start it with --jinja for tool calls), vLLM, LM Studio,
    Qwen on Alibaba Cloud Model Studio (DashScope), OpenRouter.
    """
    name = "OpenAI-compatible server"
    default_base_url = "http://localhost:8080/v1"
    default_model = "local"
    requires_api_key = False

    def __init__(self, api_key: str = None, model: str = None, base_url: str = None,
                 think: bool = None, timeout: float = 120):
        if self.requires_api_key and not api_key:
            raise ValueError(f"{self.name} API key is required.")
        self.api_key = api_key
        self.model = model or self.default_model
        self.base_url = (base_url or self.default_base_url).rstrip("/")
        self.think = think
        self.timeout = timeout
        logger.info(f"{type(self).__name__} initialized: {self.base_url}, model {self.model}")

    def _think_options(self):
        """Reasoning on/off for Qwen 3.x / Gemma 4. llama-server and vLLM pass this to the chat
        template; with reasoning on, a small model can spend the whole token budget thinking."""
        if self.think is None:
            return {}
        return {"chat_template_kwargs": {"enable_thinking": bool(self.think)}}

    def _headers(self):
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        return headers

    def generate(self, prompt: str, tools: list = None, media_data: bytes = None, mime_type: str = "image/jpeg", temperature: float = 0.7, max_tokens: int = 150) -> tuple[str, dict]:
        if media_data:
            data_url = f"data:{mime_type};base64,{base64.b64encode(media_data).decode('ascii')}"
            content = [{"type": "text", "text": prompt},
                       {"type": "image_url", "image_url": {"url": data_url}}]
        else:
            content = prompt
        payload = {
            "model": self.model,
            "messages": [{"role": "user", "content": content}],
            "temperature": temperature,
            "max_tokens": max_tokens,
            "stream": False,
        }
        if tools:
            payload["tools"] = tools
            payload["tool_choice"] = "auto"
        payload.update(self._think_options())

        try:
            response = requests.post(f"{self.base_url}/chat/completions", headers=self._headers(),
                                     json=payload, timeout=self.timeout)
            response.raise_for_status()
            result = response.json()
        except requests.exceptions.Timeout:
            logger.error(f"{self.name} request timed out.")
            return f"Error: {self.name} request timed out.", {}
        except requests.exceptions.ConnectionError:
            logger.error(f"Could not connect to {self.name} at {self.base_url}. Is it running?")
            return f"Error: Could not connect to {self.name} at {self.base_url}.", {}
        except requests.exceptions.RequestException as e:
            logger.error(f"{self.name} API error: {e}")
            return f"Error: {self.name} API error: {e}", {}
        except ValueError as e:
            logger.error(f"{self.name} returned invalid JSON: {e}")
            return f"Error: Invalid response from {self.name}.", {}

        choices = result.get("choices") if isinstance(result, dict) else None
        if not choices:
            logger.error(f"{self.name} response missing choices: {result}")
            return f"Error: No response from {self.name}.", {}
        message = choices[0].get("message") or {}
        truncated = choices[0].get("finish_reason") == "length"
        for tool_call in message.get("tool_calls") or []:
            function = tool_call.get("function") or {}
            arguments = _parse_arguments(function.get("arguments"))
            if not function.get("name") or arguments is None:
                logger.error(f"{self.name} returned an invalid tool call: {tool_call}")
                if truncated:
                    return (f"Error: the answer from {self.name} was cut off at max_tokens "
                            "(reasoning models: raise llm_max_tokens or set llm_think off).", {})
                return f"Error: Invalid tool call from {self.name}.", {}
            return "", {"name": function["name"], "arguments": arguments}
        text = strip_thinking(message.get("content"))
        if not text and truncated:
            return (f"Error: {self.name} used all max_tokens before answering "
                    "(reasoning models: raise llm_max_tokens or set llm_think off).", {})
        return text, {}

    def get_model_name(self) -> str:
        return self.model


class LlamaCppClient(OpenAICompatibleClient):
    """llama.cpp llama-server. Tool calls need the server started with --jinja."""
    name = "llama.cpp server"
    default_base_url = "http://localhost:8080/v1"
    default_model = "local"   # llama-server serves one model and ignores the name


class QwenCloudClient(OpenAICompatibleClient):
    """Qwen on Alibaba Cloud Model Studio (DashScope), OpenAI-compatible mode.

    Default endpoint: Singapore. Beijing: https://dashscope.aliyuncs.com/compatible-mode/v1,
    US: https://dashscope-us.aliyuncs.com/compatible-mode/v1.
    """
    name = "Qwen (DashScope)"
    default_base_url = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
    default_model = "qwen-plus"
    requires_api_key = True

    def _think_options(self):
        # DashScope takes enable_thinking as a top-level request field
        return {} if self.think is None else {"enable_thinking": bool(self.think)}


class OpenRouterClient(OpenAICompatibleClient):
    """LLM client for OpenRouter API."""
    name = "OpenRouter"
    default_base_url = "https://openrouter.ai/api/v1"
    default_model = "openai/gpt-3.5-turbo"
    requires_api_key = True

    def _think_options(self):
        return {}   # not forwarded: OpenRouter routes to many providers with their own options

    def _headers(self):
        headers = super()._headers()
        headers["HTTP-Referer"] = "https://xpi-blocks.ai"
        headers["X-Title"] = "XPI Blocks LLM"
        return headers


DEFAULT_GEMINI_MODEL = "gemini-3.6-flash"


def openai_tools_to_gemini(tools):
    """OpenAI-style tools ({"type": "function", "function": {...}}) -> Gemini function declarations.

    Gemini takes {"name", "description", "parameters"} with an OpenAPI-subset schema; passing
    the OpenAI wrapper as is means the model never sees the functions.
    """
    declarations = []
    for tool in tools or []:
        fn = tool.get("function", tool) if isinstance(tool, dict) else None
        if not fn or "name" not in fn:
            raise ValueError(f"Not a function tool: {tool!r}")
        decl = {"name": fn["name"], "description": fn.get("description", "")}
        params = fn.get("parameters")
        if params and params.get("properties"):
            decl["parameters"] = params
        declarations.append(decl)
    return declarations


class GeminiClient(LLMClient):
    """LLM client for the Google Gemini API (google-genai SDK).

    ``client`` lets tests pass a google.genai.Client with a fake transport.
    """
    def __init__(self, api_key: str, model: str = DEFAULT_GEMINI_MODEL, client=None):
        try:
            from google import genai
            from google.genai import types
        except ImportError:
            raise ImportError("Please install google-genai: pip install google-genai")
        self.types = types
        if client is None:
            if not api_key:
                raise ValueError("Gemini API key is required.")
            client = genai.Client(api_key=api_key)
        self.client = client
        self.model = model or DEFAULT_GEMINI_MODEL
        logger.info(f"GeminiClient initialized with model: {self.model}")

    def generate(self, prompt: str, tools: list = None, media_data: bytes = None, mime_type: str = "image/jpeg", temperature: float = 0.7, max_tokens: int = 150) -> tuple[str, dict]:
        t = self.types
        contents = [prompt]
        if media_data:
            contents.append(t.Part.from_bytes(data=media_data, mime_type=mime_type))

        config = t.GenerateContentConfig(temperature=temperature, max_output_tokens=max_tokens)
        if tools:
            config.tools = [t.Tool(function_declarations=openai_tools_to_gemini(tools))]
            # We execute tools ourselves, after validating the arguments
            config.automatic_function_calling = t.AutomaticFunctionCallingConfig(disable=True)

        try:
            response = self.client.models.generate_content(
                model=self.model, contents=contents, config=config)
        except Exception as e:
            logger.error(f"Gemini API error: {e}")
            return f"Error: Gemini API error: {e}", {}

        calls = response.function_calls or []
        if calls:
            call = calls[0]
            return "", {"name": call.name, "arguments": dict(call.args or {})}
        text = response.text
        if text:
            return text, {}
        feedback = getattr(response, "prompt_feedback", None)
        if feedback is not None and getattr(feedback, "block_reason", None):
            return f"Error: Gemini blocked the request ({feedback.block_reason}).", {}
        return "Error: No valid response from Gemini.", {}

    def get_model_name(self) -> str:
        return self.model

DEFAULT_OLLAMA_MODEL = "qwen3.5:4b"   # tools + vision; gemma4:e2b / gemma4:e4b also work


class OllamaClient(LLMClient):
    """LLM client for Ollama (local LLMs), /api/chat.

    Native tool calling for models that support it (Qwen 3.x, Gemma 4, Llama 3.x...). For a
    model without tool support Ollama answers HTTP 400; the client then falls back to asking
    for a JSON {"tool_call": ...} object in the prompt.

    ``think``: None = model default; False turns reasoning off (faster on a Raspberry Pi),
    True turns it on. The reasoning text is never returned as the answer.
    """
    def __init__(self, host: str = "http://localhost:11434", model: str = DEFAULT_OLLAMA_MODEL,
                 think: bool = None, timeout: float = 120):
        self.host = (host or "http://localhost:11434").rstrip("/")
        self.model = model or DEFAULT_OLLAMA_MODEL
        self.think = think
        self.timeout = timeout
        self.api_url = f"{self.host}/api/chat"
        self.native_tools = True   # set to False after the server says the model has no tools
        logger.info(f"OllamaClient initialized with host: {self.host}, model: {self.model}.")

    @staticmethod
    def _parse_tool_call(text):
        """The {"tool_call": {...}} object in a reply, also inside a ```json fence; else None."""
        start = text.find("{")
        while start != -1:
            try:
                obj, _ = json.JSONDecoder().raw_decode(text[start:])
            except json.JSONDecodeError:
                start = text.find("{", start + 1)
                continue
            call = obj.get("tool_call") if isinstance(obj, dict) else None
            if isinstance(call, dict) and isinstance(call.get("name"), str) \
                    and isinstance(call.get("arguments", {}), dict):
                return {"name": call["name"], "arguments": call.get("arguments", {})}
            start = text.find("{", start + 1)
        return None

    @staticmethod
    def _prompt_with_tools(prompt, tools):
        tool_definitions = json.dumps([t.get('function', t) for t in tools], indent=2)
        return (
            f"{prompt}\n\n"
            f"You have access to the following tools:\n```json\n{tool_definitions}\n```\n"
            f"To use a tool, respond with a JSON object like this: "
            f"`{{\"tool_call\": {{\"name\": \"tool_name\", \"arguments\": {{\"arg1\": \"value1\"}}}}}}`\n"
            f"If you don't need to use a tool, respond normally."
        )

    def _post(self, payload):
        """(result dict, None) or (None, error text). HTTP 400 'does not support tools' -> 'NO_TOOLS'."""
        try:
            response = requests.post(self.api_url, json=payload, timeout=self.timeout)
            if response.status_code == 400 and "tools" in payload and \
                    "does not support tools" in (getattr(response, "text", "") or ""):
                return None, "NO_TOOLS"
            response.raise_for_status()
            return response.json(), None
        except requests.exceptions.Timeout:
            logger.error("Ollama request timed out.")
            return None, "Error: Ollama request timed out."
        except requests.exceptions.ConnectionError:
            logger.error(f"Could not connect to Ollama server at {self.host}. Is it running?")
            return None, f"Error: Could not connect to Ollama server at {self.host}."
        except requests.exceptions.RequestException as e:
            logger.error(f"Ollama API error: {e}")
            return None, f"Error: Ollama API error: {e}"
        except ValueError as e:   # body is not JSON
            logger.error(f"Ollama returned invalid JSON: {e}")
            return None, "Error: Invalid response from Ollama."

    def generate(self, prompt: str, tools: list = None, media_data: bytes = None, mime_type: str = "image/jpeg", temperature: float = 0.7, max_tokens: int = 150) -> tuple[str, dict]:
        message = {"role": "user", "content": prompt}
        if media_data:
            message["images"] = [base64.b64encode(media_data).decode("ascii")]
        payload = {
            "model": self.model,
            "messages": [message],
            "options": {"temperature": temperature, "num_predict": max_tokens},
            "stream": False,
        }
        if self.think is not None:
            payload["think"] = bool(self.think)

        native = bool(tools) and self.native_tools
        if native:
            payload["tools"] = tools
        elif tools:
            message["content"] = self._prompt_with_tools(prompt, tools)

        result, error = self._post(payload)
        if error == "NO_TOOLS":
            logger.warning(f"Ollama model {self.model} has no native tool support; "
                           "falling back to JSON tool calls in the prompt.")
            self.native_tools = False
            return self.generate(prompt, tools, media_data, mime_type, temperature, max_tokens)
        if error:
            return error, {}

        reply = result.get("message") if isinstance(result, dict) else None
        if not isinstance(reply, dict):
            logger.error(f"Ollama response missing 'message': {result}")
            return "Error: No response from Ollama.", {}
        for tool_call in reply.get("tool_calls") or []:
            function = tool_call.get("function") or {}
            arguments = _parse_arguments(function.get("arguments"))
            if function.get("name") and arguments is not None:
                return "", {"name": function["name"], "arguments": arguments}
        text = strip_thinking(reply.get("content"))
        if tools and not native:
            call = self._parse_tool_call(text)
            if call:
                return "", call
        if not text:
            return "Error: No response from Ollama.", {}
        return text, {}

    def get_model_name(self) -> str:
        return self.model


CLIENT_TYPES = {
    "gemini": "Google Gemini API (Gemini and Gemma 4 models)",
    "ollama": "Ollama, local (Qwen 3.x, Gemma 4, ...)",
    "llamacpp": "llama.cpp llama-server, local (OpenAI-compatible, start with --jinja)",
    "openai": "any OpenAI-compatible server (vLLM, LM Studio, ...): set the base URL",
    "qwen": "Qwen on Alibaba Cloud Model Studio (DashScope)",
    "openrouter": "OpenRouter",
}
_ALIASES = {"llama.cpp": "llamacpp", "llama_cpp": "llamacpp", "llama-cpp": "llamacpp",
            "openai_compatible": "openai", "dashscope": "qwen"}
_KEY_ENV = {"gemini": "GEMINI_API_KEY", "openrouter": "OPENROUTER_API_KEY",
            "qwen": "DASHSCOPE_API_KEY", "openai": "OPENAI_API_KEY", "llamacpp": "LLAMA_API_KEY"}


def parse_think(value):
    """ROS parameter 'auto' / 'on' / 'off' (also true/false) -> None / True / False."""
    v = str(value).strip().lower()
    if v in ("on", "true", "1", "yes"):
        return True
    if v in ("off", "false", "0", "no"):
        return False
    return None


def llm_client_factory(client_type: str, api_key: str = None, model: str = None, host: str = None,
                       think: bool = None) -> LLMClient:
    """Create an LLM client.

    client_type: one of CLIENT_TYPES (aliases: llama.cpp, dashscope, openai_compatible).
    api_key: empty -> the provider's environment variable (GEMINI_API_KEY, DASHSCOPE_API_KEY,
             OPENROUTER_API_KEY, OPENAI_API_KEY, LLAMA_API_KEY).
    model, host: empty -> the client's default. ``host`` is the server URL for ollama,
             and the API base URL (.../v1) for llamacpp, openai, qwen and openrouter.
    think: reasoning on/off for Qwen 3.x / Gemma 4 (None = model default). Ollama: "think";
             llama.cpp / OpenAI-compatible: chat_template_kwargs.enable_thinking; DashScope:
             enable_thinking. Ignored for gemini and openrouter.
    """
    kind = (client_type or "").strip().lower()
    kind = _ALIASES.get(kind, kind)
    if kind not in CLIENT_TYPES:
        raise ValueError(f"Unknown LLM client type: {client_type}. Use one of: {', '.join(CLIENT_TYPES)}")
    api_key = api_key or os.environ.get(_KEY_ENV.get(kind, ""), "") or None
    opts = {"model": model} if model else {}
    if kind == "gemini":
        return GeminiClient(api_key=api_key, **opts)
    if kind == "ollama":
        return OllamaClient(**({"host": host} if host else {}), think=think, **opts)
    cls = {"llamacpp": LlamaCppClient, "openai": OpenAICompatibleClient,
           "qwen": QwenCloudClient, "openrouter": OpenRouterClient}[kind]
    return cls(api_key=api_key, base_url=host or None, think=think, **opts)
