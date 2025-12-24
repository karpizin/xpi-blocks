# XPI-Blocks: Manual Installation Guide

Follow these steps if you want to run XPI-Blocks directly on your host OS (Ubuntu 22.04 recommended) without Docker.

## 📋 Prerequisites
1.  **Raspberry Pi 4 or 5**.
2.  **Ubuntu 22.04 LTS** (64-bit).
3.  **ROS2 Humble** installed.

## 🚀 Installation Steps

1.  **Clone the repository:**
    ```bash
    git clone git@github.com:karpizin/xpi-blocks.git
    cd xpi-blocks
    ```
2.  **Install system dependencies:**
    ```bash
    sudo apt-get update
    sudo apt-get install -y python3-gpiozero python3-pip libgpiod2
    ```
3.  **Install ROS2 dependencies:**
    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
4.  **Build the workspace:**
    ```bash
    colcon build --symlink-install
    ```
5.  **Source the workspace:**
    ```bash
    . install/setup.bash
    ```

Now you can run the example nodes and launch files!
