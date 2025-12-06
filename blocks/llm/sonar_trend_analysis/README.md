# Sonar Trend Analysis with LLM

This block demonstrates how to use a Large Language Model (LLM) to analyze a stream of sonar data and identify trends (e.g., approaching, moving away, stationary). It supports various LLM providers, including Google Gemini, OpenRouter, and local Ollama models.

## 📦 Bill of Materials
*   Raspberry Pi with a running `xpi_sensors/sonar_node`.
*   Internet connection (for cloud LLMs like Gemini/OpenRouter) OR a local Ollama server running a compatible model (e.g., `ollama run llama2`).
*   An API key for cloud LLM providers (e.g., `GEMINI_API_KEY`, `OPENROUTER_API_KEY`).

## 🚀 Quick Start

### 1. Launch the Sonar Node (from `xpi_sensors` package)
First, ensure you have a sonar node running and publishing data on `/sonar_front/range`. If you don't have physical hardware, you can run it in mock mode:
```bash
ros2 launch xpi_sensors sonar.launch.py mock_hardware:=true
```

### 2. Prepare LLM Environment
*   **For Gemini API:** Obtain a `GEMINI_API_KEY` from Google AI Studio. Set it as an environment variable:
    ```bash
    export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
    ```
*   **For OpenRouter API:** Obtain an `OPENROUTER_API_KEY` from OpenRouter.ai. Set it as an environment variable:
    ```bash
    export OPENROUTER_API_KEY="YOUR_OPENROUTER_API_KEY"
    ```
*   **For Ollama (Local LLM):**
    *   Install Ollama on your system: `curl -fsSL https://ollama.com/install.sh | sh`
    *   Pull a model, e.g., Llama 2: `ollama pull llama2`
    *   Run the Ollama server (it usually runs in the background after installation).
    *   The `xpi_llm` node will connect to `http://localhost:11434` by default.

### 3. Launch the Sonar Trend Analyzer Node
Choose your desired LLM client:

*   **Using Gemini (cloud):**
    ```bash
    export GEMINI_API_KEY="YOUR_GEMINI_API_KEY" # Make sure this is set!
    ros2 launch xpi_llm sonar_trend_analyzer.launch.py llm_client_type:=gemini llm_model:=gemini-pro
    ```
*   **Using OpenRouter (cloud, with e.g. Mistral 7B):**
    ```bash
    export OPENROUTER_API_KEY="YOUR_OPENROUTER_API_KEY" # Make sure this is set!
    ros2 launch xpi_llm sonar_trend_analyzer.launch.py llm_client_type:=openrouter llm_model:=mistralai/mistral-7b-instruct-v0.2
    ```
*   **Using Ollama (local, with e.g. Llama 2):**
    ```bash
    # Ensure Ollama server is running and `llama2` model is pulled
    ros2 launch xpi_llm sonar_trend_analyzer.launch.py llm_client_type:=ollama llm_model:=llama2 ollama_host:=http://localhost:11434
    ```

## 📡 Interface
### Subscribers
*   `sonar_topic` (`sensor_msgs/Range`): Input sonar data. Default: `/sonar_front/range`.

### Publishers
*   `~/sonar_trend` (`std_msgs/String`): The LLM's textual analysis of the sonar trend.

### Parameters
*   `llm_client_type` (string, default: `gemini`): Which LLM backend to use (`openrouter`, `gemini`, `ollama`).
*   `llm_api_key` (string, default: `''`): API key for cloud LLMs (e.g., from `GEMINI_API_KEY` env var).
*   `llm_model` (string, default: `gemini-pro`): Specific model name (e.g., `gemini-pro`, `llama2`, `openai/gpt-3.5-turbo`).
*   `ollama_host` (string, default: `http://localhost:11434`): URL of your local Ollama server.
*   `history_size` (int, default: `20`): Number of past sonar readings to send to the LLM for analysis.
*   `analysis_interval_sec` (float, default: `5.0`): How often the LLM is prompted for analysis.
*   `llm_temperature` (float, default: `0.5`): Creativity/randomness of LLM response (0.0-1.0).
*   `llm_max_tokens` (int, default: `100`): Maximum length of LLM's response.
*   `sonar_topic` (string, default: `/sonar_front/range`): The ROS2 topic from which to read sonar data.

## ✅ Verification
1.  Launch `sonar.launch.py` (either with hardware or `mock_hardware:=true`).
2.  Launch `sonar_trend_analyzer.launch.py` with your chosen LLM configuration.
3.  In a new terminal, monitor the LLM's output:
    ```bash
    ros2 topic echo /sonar_trend_analyzer/sonar_trend
    ```
    You should see periodic text descriptions of the simulated (or real) sonar data trend.

## ⚠️ Troubleshooting
*   **"Failed to initialize LLM client" / API Key errors:** Ensure your API key is correctly set as an environment variable or passed directly, and that you have an internet connection for cloud LLMs.
*   **"Could not connect to Ollama server"**: Make sure Ollama is installed, running, and the specified model is pulled.
*   **No output on `/sonar_trend`**: Check if `sonar_node` is publishing data and if there are errors in the `sonar_trend_analyzer` node's logs (use `output='screen'` in its launch file or `ros2 run` directly).
