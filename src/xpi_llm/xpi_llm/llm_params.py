"""The LLM parameters shared by the xpi_llm nodes, and the client built from them.

llm_client_type  gemini | ollama | llamacpp | openai | qwen | openrouter
llm_model        '' = the client's default (Gemini: gemini-3.6-flash, Ollama: qwen3.5:4b)
llm_api_key      '' = the provider's environment variable (GEMINI_API_KEY, DASHSCOPE_API_KEY...)
llm_base_url     '' = the client's default; the server for ollama / llamacpp / openai
                 (e.g. http://192.168.1.20:8080/v1 for llama-server on another machine)
ollama_host      kept for existing launch files; used for ollama when llm_base_url is empty
llm_think        auto | on | off: reasoning of Qwen 3.x / Gemma 4 (ollama, llamacpp, openai, qwen);
                 off = faster, and a small model cannot spend max_tokens on thinking
"""
from xpi_llm.llm_clients import llm_client_factory, parse_think

LLM_PARAMETERS = {
    'llm_client_type': 'gemini',
    'llm_api_key': '',
    'llm_model': '',
    'llm_base_url': '',
    'ollama_host': 'http://localhost:11434',
    'llm_think': 'auto',
}


def declare_llm_parameters(node, **defaults):
    for name, value in LLM_PARAMETERS.items():
        node.declare_parameter(name, defaults.get(name, value))


def llm_client_from_parameters(node):
    """Build the client; raises ValueError/ImportError like llm_client_factory."""
    p = {name: node.get_parameter(name).value for name in LLM_PARAMETERS}
    kind = str(p['llm_client_type'])
    host = p['llm_base_url'] or (p['ollama_host'] if kind.lower() == 'ollama' else '')
    return llm_client_factory(kind, api_key=p['llm_api_key'], model=p['llm_model'],
                              host=host, think=parse_think(p['llm_think']))
