import os
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

class OpenRouterClient(LLMClient):
    """LLM client for OpenRouter API."""
    def __init__(self, api_key: str, model: str = "openai/gpt-3.5-turbo"):
        if not api_key:
            raise ValueError("OpenRouter API key is required.")
        self.api_key = api_key
        self.model = model
        self.base_url = "https://openrouter.ai/api/v1/chat/completions"
        logger.info(f"OpenRouterClient initialized with model: {self.model}")

    def generate(self, prompt: str, tools: list = None, media_data: bytes = None, mime_type: str = "image/jpeg", temperature: float = 0.7, max_tokens: int = 150) -> tuple[str, dict]:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "HTTP-Referer": "https://xpi-blocks.ai", 
            "X-Title": "XPI Blocks LLM",
            "Content-Type": "application/json"
        }
        
        # Build message content
        if media_data:
            import base64
            base64_data = base64.b64encode(media_data).decode('utf-8')
            content = [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url", # OpenRouter mostly supports images via image_url
                    "image_url": {
                        "url": f"data:{mime_type};base64,{base64_data}"
                    }
                }
            ]
        else:
            content = prompt

        messages = [{"role": "user", "content": content}]
        
        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature,
            "max_tokens": max_tokens,
            "stream": False 
        }
        
        if tools:
            payload["tools"] = tools
            payload["tool_choice"] = "auto"

        try:
            response = requests.post(self.base_url, headers=headers, json=payload, timeout=60) # Increased timeout for images
            response.raise_for_status()
            result = response.json()
            if result and result.get("choices"):
                choice = result["choices"][0]
                message = choice["message"]
                
                tool_calls = message.get("tool_calls")
                if tool_calls:
                    tool_call = tool_calls[0]
                    function_call = tool_call.get("function")
                    if function_call:
                        try:
                            arguments = json.loads(function_call.get("arguments") or "{}")
                        except (TypeError, ValueError):
                            logger.error(f"OpenRouter returned invalid tool arguments: {function_call}")
                            return "Error: Invalid tool call from OpenRouter.", {}
                        return "", {"name": function_call["name"], "arguments": arguments}
                
                return message.get("content") or "", {}
            else:
                logger.error(f"OpenRouter response missing choices: {result}")
                return "Error: No response from OpenRouter.", {}
        except requests.exceptions.Timeout:
            logger.error("OpenRouter request timed out.")
            return "Error: OpenRouter request timed out.", {}
        except requests.exceptions.RequestException as e:
            logger.error(f"OpenRouter API error: {e}")
            return f"Error: OpenRouter API error: {e}", {}

    def get_model_name(self) -> str:
        return self.model

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

class OllamaClient(LLMClient):
    """LLM client for Ollama (local LLMs)."""
    # Ollama currently does not natively support complex function calling in its base API.
    # We will simulate it by prompting the model to output JSON.
    def __init__(self, host: str = "http://localhost:11434", model: str = "llava"): # Defaulting to llava for vision
        self.host = (host or "http://localhost:11434").rstrip("/")
        self.model = model or "llava"
        self.api_url = f"{self.host}/api/generate"
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

    def generate(self, prompt: str, tools: list = None, media_data: bytes = None, mime_type: str = "image/jpeg", temperature: float = 0.7, max_tokens: int = 150) -> tuple[str, dict]:
        full_prompt = prompt
        
        data = {
            "model": self.model,
            "prompt": full_prompt,
            "options": {
                "temperature": temperature,
                "num_predict": max_tokens
            },
            "stream": False
        }

        if media_data:
            import base64
            # Ollama expects media as base64 string array
            base64_data = base64.b64encode(media_data).decode('utf-8')
            data["images"] = [base64_data] # Ollama uses 'images' field for multimodal data

        if tools and not media_data: 
            # For Ollama, we inject the tool definitions into the prompt and instruct the model to use them.
            tool_definitions = json.dumps([t.get('function', t) for t in tools], indent=2)
            full_prompt = (
                f"{prompt}\n\n"
                f"You have access to the following tools:\n```json\n{tool_definitions}\n```\n"
                f"To use a tool, respond with a JSON object like this: "
                f"`{{\"tool_call\": {{\"name\": \"tool_name\", \"arguments\": {{\"arg1\": \"value1\", \"arg2\": \"value2\"}}}}}}`\n"
                f"If you don't need to use a tool, respond normally."
            )
            data["prompt"] = full_prompt
        
        try:
            response = requests.post(self.api_url, json=data, timeout=120)
            response.raise_for_status()
            result = response.json()
        except requests.exceptions.Timeout:
            logger.error("Ollama request timed out.")
            return "Error: Ollama request timed out.", {}
        except requests.exceptions.ConnectionError:
            logger.error(f"Could not connect to Ollama server at {self.host}. Is it running?")
            return f"Error: Could not connect to Ollama server at {self.host}.", {}
        except requests.exceptions.RequestException as e:
            logger.error(f"Ollama API error: {e}")
            return f"Error: Ollama API error: {e}", {}
        except ValueError as e:   # body is not JSON
            logger.error(f"Ollama returned invalid JSON: {e}")
            return "Error: Invalid response from Ollama.", {}

        llm_response_text = result.get("response") if isinstance(result, dict) else None
        if not llm_response_text:
            logger.error(f"Ollama response missing 'response' field: {result}")
            return "Error: No response from Ollama.", {}
        if tools:
            call = self._parse_tool_call(llm_response_text)
            if call:
                return "", call
        return llm_response_text, {}

    def get_model_name(self) -> str:
        return self.model

def llm_client_factory(client_type: str, api_key: str = None, model: str = None, host: str = None) -> LLMClient:
    """Factory function to create an LLM client based on type.

    ``model``/``host`` left as None (or '') keep each client's own default.
    """
    kind = (client_type or "").lower()
    opts = {"model": model} if model else {}
    if kind == "openrouter":
        return OpenRouterClient(api_key=api_key, **opts)
    elif kind == "gemini":
        return GeminiClient(api_key=api_key, **opts)
    elif kind == "ollama":
        return OllamaClient(**({"host": host} if host else {}), **opts)
    else:
        raise ValueError(f"Unknown LLM client type: {client_type}")

# Example usage (for testing purposes, not run in ROS2 node directly)
if __name__ == "__main__":
    # --- OpenRouter Example ---
    try:
        openrouter_key = os.getenv("OPENROUTER_API_KEY")
        if openrouter_key:
            logger.info("\n--- Testing OpenRouter Client ---")
            openrouter_llm = llm_client_factory("openrouter", api_key=openrouter_key, model="mistralai/mistral-7b-instruct-v0.2")
            
            tools_example = [
                {
                    "type": "function",
                    "function": {
                        "name": "set_relay_state",
                        "description": "Sets the state of a specific relay.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "relay_id": {"type": "integer", "description": "The ID of the relay to control (0-15)."},
                                "state": {"type": "boolean", "description": "The desired state: true for ON, false for OFF."}
                            },
                            "required": ["relay_id", "state"]
                        }
                    }
                }
            ]
            
            text_response, tool_call = openrouter_llm.generate("Turn on relay 5.", tools=tools_example)
            logger.info(f"OpenRouter Tool Call ({openrouter_llm.get_model_name()}): {tool_call}")
            logger.info(f"OpenRouter Text Response ({openrouter_llm.get_model_name()}): {text_response}")

            text_response, tool_call = openrouter_llm.generate("Tell me a joke.")
            logger.info(f"OpenRouter Joke ({openrouter_llm.get_model_name()}): {text_response}")

        else:
            logger.warning("OPENROUTER_API_KEY not set, skipping OpenRouter test.")
    except Exception as e:
        logger.error(f"OpenRouter test failed: {e}")

    # --- Gemini Example ---
    try:
        gemini_key = os.getenv("GEMINI_API_KEY")
        if gemini_key:
            logger.info("\n--- Testing Gemini Client ---")
            gemini_llm = llm_client_factory("gemini", api_key=gemini_key)
            
            text_response, tool_call = gemini_llm.generate("Turn on relay 5.", tools=tools_example)
            logger.info(f"Gemini Tool Call ({gemini_llm.get_model_name()}): {tool_call}")
            logger.info(f"Gemini Text Response ({gemini_llm.get_model_name()}): {text_response}")

            text_response, tool_call = gemini_llm.generate("Tell me a fun fact about robots.")
            logger.info(f"Gemini Fun Fact ({gemini_llm.get_model_name()}): {text_response}")
        else:
            logger.warning("GEMINI_API_KEY not set, skipping Gemini test.")
    except Exception as e:
        logger.error(f"Gemini test failed: {e}")

    # --- Ollama Example ---
    try:
        logger.info("\n--- Testing Ollama Client ---")
        ollama_llm = llm_client_factory("ollama", host="http://localhost:11434", model="llama2")
        text_response, tool_call = ollama_llm.generate("Turn on relay 5.", tools=tools_example)
        logger.info(f"Ollama Tool Call ({ollama_llm.get_model_name()}): {tool_call}")
        logger.info(f"Ollama Text Response ({ollama_llm.get_model_name()}): {text_response}")
        
        text_response, tool_call = ollama_llm.generate("Why is the sky blue?")
        logger.info(f"Ollama Why is the sky blue? ({ollama_llm.get_model_name()}): {text_response}")

    except Exception as e:
        logger.error(f"Ollama test failed: {e}. Make sure Ollama server is running and model 'llama2' is pulled.")

