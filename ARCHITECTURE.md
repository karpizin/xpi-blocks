# XPI-Blocks: Architectural Manifesto

Этот документ определяет технические стандарты и архитектурные решения проекта `xpi-blocks`.

## 1. Организация воркспейса и пакетов
Мы используем **структуру пакетов по категориям**, чтобы минимизировать зависимости.

```text
src/
├── xpi_interfaces/       # Кастомные .msg, .srv, .action файлы
├── xpi_commons/          # Общие утилиты, HAL, моки
├── xpi_inputs/           # Джойстики, РУ, клавиатуры
├── xpi_sensors/          # Лидары, IMU, камеры, датчики среды
├── xpi_actuators/        # Моторы, сервоприводы, реле
├── xpi_llm/              # Интеграция LLM/VLM, Tool Calling
└── xpi_navigation/       # Алгоритмы SLAM и позиционирования
```

## 2. Стандарты проектирования нод (Tiered Architecture)

*   **Tier 1: Simple Blocks**
    *   Наследуются от `rclpy.node.Node`. Фокус на простоте кода.
*   **Tier 2: Advanced Blocks**
    *   Наследуются от `rclpy.lifecycle.NodeLifecycle`. Управление состоянием и безопасностью.

## 3. Конфигурация и параметры
*   **Никакого хардкода:** Все настройки (пины, адреса) выносятся в ROS Parameters.
*   **Дескрипторы:** Параметры должны иметь описание и границы.
*   **YAML First:** Каждый блок включает пример `params.yaml`.

## 4. Hardware Abstraction Layer (HAL)
Для совместимости между Raspberry Pi и поддержки моков.
*   **Библиотеки:** `gpiozero` для GPIO и `smbus2` (через `xpi_commons`) для I2C.
*   **Mocking:** Использование `GPIOZERO_PIN_FACTORY=mock` для тестов без железа.

## 5. Интеграция с ИИ (xpi_llm)
*   **LLM Client Abstraction:** Единый интерфейс для Google Gemini, OpenRouter, Ollama.
*   **Tool Calling:** LLM интерпретирует команды и вызывает ROS2 сервисы/экшены.
*   **Context Injection:** Данные с сенсоров автоматически упаковываются в промпты.

## 6. Робототехнические проекты

### Hexapod Master
*   **Kinematics Core**: Реализация обратной кинематики (IK) для 6 ног.
*   **Gait Engine**: Генерация походок (Tripod, Wave, Ripple) с плавными переходами.
*   **Auto-Leveling**: Стабилизация корпуса на основе данных IMU.

### Swarm & Mesh
*   **LoRa Mesh**: Связь через **Meshtastic** (пакет `xpi_comms`).
*   **Consensus Engine**: Децентрализованное голосование за режим миссии без центрального сервера.

## 7. Система индикации (USIS)
Universal Status Indication System (USIS) — стандарт световых сигналов:
*   `Blue Pulsing`: Инициализация / Ожидание.
*   `Green Constant`: Готовность к работе.
*   `Red Blinking`: Критическая ошибка.
*   `Yellow Rotating`: Работа ИИ / Анализ.