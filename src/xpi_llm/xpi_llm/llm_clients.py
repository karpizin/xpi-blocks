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
    def generate(self, prompt: str, tools: list = None, image_data: bytes = None, **kwargs) -> tuple[str, dict]:
        """
        Sends a prompt to the LLM and returns the generated text and/or a tool_call.
        
        Args:
            prompt: The text prompt.
            tools: Optional list of tool definitions.
            image_data: Optional raw bytes of an image (e.g. JPEG encoded) for VLM tasks.
            
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

    def generate(self, prompt: str, tools: list = None, image_data: bytes = None, temperature: float = 0.7, max_tokens: int = 150) -> tuple[str, dict]:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "HTTP-Referer": "https://xpi-blocks.ai", 
            "X-Title": "XPI Blocks LLM",
            "Content-Type": "application/json"
        }
        
        # Build message content
        if image_data:
            import base64
            base64_image = base64.b64encode(image_data).decode('utf-8')
            content = [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{base64_image}"
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
                        return "", {"name": function_call["name"], "arguments": json.loads(function_call["arguments"])}
                
                return message.get("content", ""), {}
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

class GeminiClient(LLMClient):
    """LLM client for Google Gemini API."""
    def __init__(self, api_key: str, model: str = "gemini-1.5-flash"): # Defaulting to a VLM capable model
        try:
            import google.generativeai as genai
            from google.generativeai.types import HarmCategory, HarmBlockThreshold
            self.genai = genai
            self.HarmCategory = HarmCategory
            self.HarmBlockThreshold = HarmBlockThreshold
        except ImportError:
            raise ImportError("Please install google-generativeai: pip install google-generativeai")
        
        if not api_key:
            raise ValueError("Gemini API key is required.")
        self.genai.configure(api_key=api_key)
        self.model = model
        logger.info(f"GeminiClient initialized with model: {self.model}")

    def generate(self, prompt: str, tools: list = None, image_data: bytes = None, temperature: float = 0.7, max_tokens: int = 150) -> tuple[str, dict]:
        model = self.genai.GenerativeModel(self.model)
        
        generation_config = self.genai.types.GenerationConfig(
            temperature=temperature,
            max_output_tokens=max_tokens,
        )
        
        safety_settings = {
            self.HarmCategory.HARM_CATEGORY_HARASSMENT: self.HarmBlockThreshold.BLOCK_NONE,
            self.HarmCategory.HARM_CATEGORY_HATE_SPEECH: self.HarmBlockThreshold.BLOCK_NONE,
            self.HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: self.HarmBlockThreshold.BLOCK_NONE,
            self.HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: self.HarmBlockThreshold.BLOCK_NONE,
        }

        content = [prompt]
        if image_data:
            # Gemini expects 'data' and 'mime_type' for blobs
            image_blob = {
                "mime_type": "image/jpeg",
                "data": image_data
            }
            content.append(image_blob)

        try:
            if tools:
                response = model.generate_content(
                    content,
                    tools=tools,
                    tool_config=self.genai.types.ToolConfig(function_calling_config=self.genai.types.FunctionCallingConfig(mode='AUTO')),
                    generation_config=generation_config,
                    safety_settings=safety_settings
                )
            else:
                response = model.generate_content(
                    content,
                    generation_config=generation_config,
                    safety_settings=safety_settings
                )

            if response.candidates:
                candidate = response.candidates[0]
                if candidate.function_calls:
                    func_call = candidate.function_calls[0]
                    return "", {"name": func_call.name, "arguments": func_call.args}
                
                if candidate.content and candidate.content.parts:
                    return candidate.content.parts[0].text, {}
            
            return "Error: No valid response from Gemini.", {}

        except Exception as e:
            logger.error(f"Gemini API error: {e}")
            if "safety settings" in str(e):
                return "Error: Gemini response blocked by safety settings. Try rephrasing.", {}
            return f"Error: Gemini API error: {e}", {}

    def get_model_name(self) -> str:
        return self.model

class OllamaClient(LLMClient):
    """LLM client for Ollama (local LLMs)."""
    # Ollama currently does not natively support complex function calling in its base API.
    # We will simulate it by prompting the model to output JSON.
    def __init__(self, host: str = "http://localhost:11434", model: str = "llava"): # Defaulting to llava for vision
        self.host = host
        self.model = model
        self.api_url = f"{self.host}/api/generate"
        logger.info(f"OllamaClient initialized with host: {self.host}, model: {self.model}.")

    def generate(self, prompt: str, tools: list = None, image_data: bytes = None, temperature: float = 0.7, max_tokens: int = 150) -> tuple[str, dict]:
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

        if image_data:
            import base64
            # Ollama expects images as base64 string array
            base64_image = base64.b64encode(image_data).decode('utf-8')
            data["images"] = [base64_image]

        if tools and not image_data: # Tool calling with vision models is tricky in Ollama, skipping logic for simplicity if image present
            # For Ollama, we inject the tool definitions into the prompt and instruct the model to use them.
            tool_definitions = json.dumps([t['function'] for t in tools], indent=2)
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
            if result and result.get("response"):
                llm_response_text = result["response"]
                # Try to parse if it's a tool call (only if we injected tools)
                if tools:
                    try:
                        parsed_json = json.loads(llm_response_text)
                        if "tool_call" in parsed_json and "name" in parsed_json["tool_call"] and "arguments" in parsed_json["tool_call"]:
                            return "", parsed_json["tool_call"]
                    except json.JSONDecodeError:
                        pass 
                
                return llm_response_text, {}
            else:
                logger.error(f"Ollama response missing 'response' field: {result}")
                return "Error: No response from Ollama.", {}
        except requests.exceptions.Timeout:
            logger.error("Ollama request timed out.")
            return "Error: Ollama request timed out.", {}
        except requests.exceptions.ConnectionError:
            logger.error(f"Could not connect to Ollama server at {self.host}. Is it running?")
            return f"Error: Could not connect to Ollama server at {self.host}."
        except requests.exceptions.RequestException as e:
            logger.error(f"Ollama API error: {e}")
            return f"Error: Ollama API error: {e}", {}

    def get_model_name(self) -> str:
        return self.model

def llm_client_factory(client_type: str, api_key: str = None, model: str = None, host: str = None) -> LLMClient:
    """Factory function to create an LLM client based on type."""
    if client_type.lower() == "openrouter":
        return OpenRouterClient(api_key=api_key, model=model)
    elif client_type.lower() == "gemini":
        return GeminiClient(api_key=api_key, model=model)
    elif client_type.lower() == "ollama":
        return OllamaClient(host=host, model=model)
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

