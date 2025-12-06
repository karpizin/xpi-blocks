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
    def generate(self, prompt: str, **kwargs) -> str:
        """Sends a prompt to the LLM and returns the generated text."""
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

    def generate(self, prompt: str, temperature: float = 0.7, max_tokens: int = 150) -> str:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "HTTP-Referer": "https://xpi-blocks.ai", # Replace with your app's actual name
            "X-Title": "XPI Blocks LLM", # Replace with your app's actual name
            "Content-Type": "application/json"
        }
        data = {
            "model": self.model,
            "messages": [{"role": "user", "content": prompt}],
            "temperature": temperature,
            "max_tokens": max_tokens
        }
        try:
            response = requests.post(self.base_url, headers=headers, json=data, timeout=30)
            response.raise_for_status()
            result = response.json()
            if result and result.get("choices"):
                return result["choices"][0]["message"]["content"]
            else:
                logger.error(f"OpenRouter response missing choices: {result}")
                return "Error: No response from OpenRouter."
        except requests.exceptions.Timeout:
            logger.error("OpenRouter request timed out.")
            return "Error: OpenRouter request timed out."
        except requests.exceptions.RequestException as e:
            logger.error(f"OpenRouter API error: {e}")
            return f"Error: OpenRouter API error: {e}"

    def get_model_name(self) -> str:
        return self.model

class GeminiClient(LLMClient):
    """LLM client for Google Gemini API."""
    def __init__(self, api_key: str, model: str = "gemini-pro"):
        try:
            import google.generativeai as genai
            self.genai = genai
        except ImportError:
            raise ImportError("Please install google-generativeai: pip install google-generativeai")
        
        if not api_key:
            raise ValueError("Gemini API key is required.")
        self.genai.configure(api_key=api_key)
        self.model = model
        logger.info(f"GeminiClient initialized with model: {self.model}")

    def generate(self, prompt: str, temperature: float = 0.7, max_tokens: int = 150) -> str:
        try:
            model = self.genai.GenerativeModel(self.model)
            response = model.generate_content(
                prompt,
                generation_config=self.genai.types.GenerationConfig(
                    temperature=temperature,
                    max_output_tokens=max_tokens,
                ),
            )
            return response.text
        except Exception as e:
            logger.error(f"Gemini API error: {e}")
            return f"Error: Gemini API error: {e}"

    def get_model_name(self) -> str:
        return self.model

class OllamaClient(LLMClient):
    """LLM client for Ollama (local LLMs)."""
    def __init__(self, host: str = "http://localhost:11434", model: str = "llama2"):
        self.host = host
        self.model = model
        self.api_url = f"{self.host}/api/generate"
        logger.info(f"OllamaClient initialized with host: {self.host}, model: {self.model}")

    def generate(self, prompt: str, temperature: float = 0.7, max_tokens: int = 150) -> str:
        data = {
            "model": self.model,
            "prompt": prompt,
            "options": {
                "temperature": temperature,
                "num_predict": max_tokens
            },
            "stream": False # We want a single response
        }
        try:
            response = requests.post(self.api_url, json=data, timeout=120)
            response.raise_for_status()
            result = response.json()
            if result and result.get("response"):
                return result["response"]
            else:
                logger.error(f"Ollama response missing 'response' field: {result}")
                return "Error: No response from Ollama."
        except requests.exceptions.Timeout:
            logger.error("Ollama request timed out.")
            return "Error: Ollama request timed out."
        except requests.exceptions.ConnectionError:
            logger.error(f"Could not connect to Ollama server at {self.host}. Is it running?")
            return f"Error: Could not connect to Ollama server at {self.host}."
        except requests.exceptions.RequestException as e:
            logger.error(f"Ollama API error: {e}")
            return f"Error: Ollama API error: {e}"

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
            response = openrouter_llm.generate("Hello, what is your name?")
            logger.info(f"OpenRouter ({openrouter_llm.get_model_name()}): {response}")
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
            response = gemini_llm.generate("Tell me a fun fact about robots.")
            logger.info(f"Gemini ({gemini_llm.get_model_name()}): {response}")
        else:
            logger.warning("GEMINI_API_KEY not set, skipping Gemini test.")
    except Exception as e:
        logger.error(f"Gemini test failed: {e}")

    # --- Ollama Example ---
    try:
        logger.info("\n--- Testing Ollama Client ---")
        # Ensure Ollama server is running (e.g., `ollama run llama2`)
        ollama_llm = llm_client_factory("ollama", host="http://localhost:11434", model="llama2")
        response = ollama_llm.generate("Why is the sky blue?")
        logger.info(f"Ollama ({ollama_llm.get_model_name()}): {response}")
    except Exception as e:
        logger.error(f"Ollama test failed: {e}. Make sure Ollama server is running and model 'llama2' is pulled.")
