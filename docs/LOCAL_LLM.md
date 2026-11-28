# LLM backends: Gemini, Gemma 4, Qwen — cloud and local

The `xpi_llm` nodes (`tool_calling_node`, `mcp_agent_node`, `vlm_observer_node`, the sonar
analyzers) talk to an LLM through one client interface. Pick the backend with
`llm_client_type` (`provider` in `vlm_observer_node`):

| `llm_client_type` | Runs | Models | Tool calling |
|---|---|---|---|
| `gemini` (default) | Google Gemini API | `gemini-3.6-flash` (default), `gemma-4-26b-a4b-it`, `gemma-4-31b-it` | native |
| `ollama` | local, Ollama | `qwen3.5:4b` (default), `qwen3.5:9b`, `gemma4:e2b`, `gemma4:e4b`, ... | native (`/api/chat`), JSON-in-prompt fallback for models without tools |
| `llamacpp` | local, llama.cpp `llama-server` | any GGUF: Qwen 3.5, Gemma 4, ... | OpenAI-style, **start the server with `--jinja`** |
| `openai` | any OpenAI-compatible server (vLLM, LM Studio, ...) | whatever it serves | OpenAI-style |
| `qwen` | Qwen on Alibaba Cloud Model Studio (DashScope) | `qwen-plus` (default), other Qwen models | OpenAI-style |
| `openrouter` | OpenRouter | any | OpenAI-style |

Common parameters (the same names in every node; `mcp_agent_node` uses `api_key`, `vlm_observer_node`
uses `provider`/`model`/`api_key`/`base_url`/`think`):

| Parameter | Default | Meaning |
|---|---|---|
| `llm_client_type` | `gemini` | Backend, see above. Aliases: `llama.cpp`, `dashscope`, `openai_compatible`. |
| `llm_model` | `''` | Empty: the backend's default model. |
| `llm_api_key` | `''` | Empty: the environment variable `GEMINI_API_KEY`, `DASHSCOPE_API_KEY`, `OPENROUTER_API_KEY`, `OPENAI_API_KEY` or `LLAMA_API_KEY`. Prefer the variable: a parameter shows in `ros2 param get`. |
| `llm_base_url` | `''` | Server URL. Ollama: `http://host:11434`; llama.cpp/OpenAI-compatible: the `.../v1` base. Empty: localhost default. |
| `llm_think` | `off` in `tool_calling_node` / `mcp_agent_node`, `auto` elsewhere | Reasoning of Qwen 3.x / Gemma 4: `auto` (model default), `on`, `off`. Sent as `think` to Ollama, `chat_template_kwargs.enable_thinking` to llama.cpp/vLLM, `enable_thinking` to DashScope. With reasoning on, a small model can use all of `llm_max_tokens` thinking and never get to the tool call; the client then says so. |

Whatever the backend, tool arguments are validated in code before anything reaches the
hardware (see `xpi_llm/tool_args.py`).

## Ollama (Qwen 3.5, Gemma 4)

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen3.5:4b        # or gemma4:e2b / gemma4:e4b
ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=ollama llm_think:=off
ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=ollama llm_model:=gemma4:e4b
```
Models without tool support in Ollama (it answers `does not support tools`) still work: the
client then describes the tools in the prompt and parses a JSON tool call from the answer.

## llama.cpp (`llama-server`)

```bash
# Qwen 3.5 0.8B (fits a Raspberry Pi easily) or Gemma 4 E2B
llama-server -hf bartowski/Qwen_Qwen3.5-0.8B-GGUF:Q4_K_M --jinja --port 8080
llama-server -hf ggml-org/gemma-4-E2B-it-GGUF:Q4_0 --jinja --port 8080
ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=llamacpp
```
`--jinja` is required: without it llama-server does not render the tools into the model's chat
template and returns no tool calls.

Checked end to end with llama-server (CPU, 2 cores) and `tool_calling_node`: Qwen 3.5 0.8B
Q4_K_M and Gemma 4 E2B Q4_0 both switch relay 5 and set PCA9685 channel 3 from plain English
commands. Gemma 4 E2B with reasoning on used all 200 tokens thinking and returned no tool call;
with `llm_think:=off` it answers in about 2 s. For images (`vlm_observer_node`), also pass the model's
`--mmproj` file. On another machine: `llm_base_url:=http://192.168.1.20:8080/v1`.

## Gemma 4 through the Gemini API

Same client as Gemini, only the model name changes:
```bash
export GEMINI_API_KEY=...
ros2 launch xpi_llm tool_calling.launch.py llm_model:=gemma-4-26b-a4b-it
```

## Qwen in the cloud (DashScope)

```bash
export DASHSCOPE_API_KEY=...
ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=qwen llm_model:=qwen-plus
```
The default endpoint is Singapore (`https://dashscope-intl.aliyuncs.com/compatible-mode/v1`);
for Beijing use `llm_base_url:=https://dashscope.aliyuncs.com/compatible-mode/v1`, for the US
`https://dashscope-us.aliyuncs.com/compatible-mode/v1`.

## Which model on which machine

* **Raspberry Pi 5 (8 GB)**: `qwen3.5:2b`/`qwen3.5:4b` or Gemma 4 E2B with `llm_think:=off`;
  expect seconds per command. Small models pick the right tool for simple commands but make
  more mistakes: the argument validation is what keeps a wrong call from reaching the hardware.
* **A PC or Jetson on the same network**: run Ollama or llama-server there, point
  `llm_base_url` at it, and use `qwen3.5:9b` or `gemma4:e4b` and larger.
* Tool calling quality depends on the runtime's chat template for the model: if a model returns
  text instead of tool calls, update Ollama / llama.cpp, or try the other runtime.
