# RTTTL Melody Library & Usage Guide

This document explains how to use the RTTTL (Ring Tone Text Transfer Language) format with the XPI-Blocks Buzzer node and provides a library of ready-to-use sounds.

## 🎵 RTTTL Format Quick Overview
An RTTTL string consists of three parts separated by colons:
`Name:Settings:Notes`

Example: `Beep:d=4,o=5,b=120:c6`
*   **Name:** Any string (max 10 chars).
*   **Settings:**
    *   `d=`: Default duration (4=quarter, 8=eighth, etc.).
    *   `o=`: Default octave (4, 5, 6).
    *   `b=`: Tempo in BPM.
*   **Notes:** A list of notes (a-g, with # for sharps, and optional duration/octave overrides). `p` is a pause.

---

## 🚀 Situational Sound Library

Use these commands to play specific sounds via `ros2 topic pub`.

### 1. System Status
*   **Startup / Success:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Success:d=4,o=5,b=160:8c6,8e6,8g6,c7'"
    ```
*   **Error / Alert:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Error:d=4,o=5,b=100:8g,8p,8g,8p,2g'"
    ```
*   **Low Battery Warning:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'LowBat:d=4,o=5,b=120:8e6,8p,8e6,8p'"
    ```

### 2. Famous Melodies
*   **Star Wars (Imperial March):**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Imperial:d=4,o=5,b=100:e,e,e,8c,16g,e,8c,16g,2e'"
    ```
*   **Terminator 2 Theme:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'T2:d=4,o=4,b=80:8f,8f,8f,2f,8f,8f,8f,2f,8f,8f,8f,2f#,8f,8f,8f,2f#'"
    ```
*   **X-Files Theme:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'XFiles:d=4,o=5,b=125:e,g,a,g,d6,2e6,p,e,g,a,g,e6,2d6'"
    ```
*   **Tron (Main Theme snippet):**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Tron:d=4,o=5,b=120:8c,8e,8g,8c6,8g,8e,8c,8p,8d,8f,8a,8d6,8a,8f,8d'"
    ```
*   **James Bond 007:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Bond:d=4,o=5,b=80:8e6,16f#6,16f#6,8f#6,8e6,8e6,8e6,8g6,16g6,16g6,8g6,8f#6,8f#6,8f#6'"
    ```
*   **Mission Impossible:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Mission:d=4,o=5,b=150:16d6,16d6,16f6,16g6,8d6,8d6,16c6,16c#6,8d6,8d6,16f6,16g6,8d6,8d6,16c6,16c#6'"
    ```
*   **Super Mario Theme:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Mario:d=4,o=5,b=125:8e6,8e6,8p,8e6,8p,8c6,8e6,8p,8g6,8p,8g5'"
    ```
*   **Nokia Tune:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Nokia:d=4,o=5,b=160:8e6,8d6,f#,g#,8b,8a,c#,e,8g#,8f#,a,b,2e6'"
    ```
*   **Indiana Jones:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Indiana:d=4,o=5,b=125:e,8f,16g,8c6,8p,d,8e,f,8p,g,8a,8b,8f6,8p,a,8b,c6,d6,e6'"
    ```

### 3. Functional Sounds
*   **Fast Beep (Proximity Warning):**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Warn:d=8,o=6,b=240:c,p,c,p,c,p,c,p'"
    ```
*   **Siren:**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Siren:d=4,o=5,b=120:2c6,2g5,2c6,2g5'"
    ```

---

## 🛠 Pro Tips
1.  **Volume:** On passive buzzers, you cannot control volume easily, but higher octaves (o=6 or o=7) often sound "louder" or more piercing.
2.  **Concurrency:** The node plays melodies in a separate thread. If you send a new melody while one is playing, the node will ignore the second one until the first finishes.
3.  **Stopping:** To stop a long melody, send an empty string or a very short "silence" melody:
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'Stop:d=4,o=5,b=100:p'"
    ```
