# Desktop Simulation Guide (Docker + Gazebo)

This guide provides a comprehensive walkthrough for setting up the **XPI-Blocks** simulation environment on a standard **amd64 Desktop/Laptop (Ubuntu)** using **Docker Compose** and **Gazebo**.

Using Docker ensures that your host system remains clean and that all ROS2 dependencies are perfectly configured.

---

## 🛠 1. Prerequisites on Host Machine
Before starting, ensure your Ubuntu host has Docker and the necessary X11 tools for GUI forwarding.

```bash
# 1. Install Docker & Compose (if not already installed)
sudo apt update && sudo apt install docker.io docker-compose-v2 -y

# 2. Install xhost to allow the container to open windows on your desktop
sudo apt install x11-xserver-utils -y

# 3. Enable local GUI access for Docker (IMPORTANT!)
xhost +local:docker
```

---

## 🚀 2. Launching with Docker Compose

We use `docker-compose.sim.yml` to automatically handle GUI forwarding, privileged access, and volume mounting.

### Build and Start
```bash
# Navigate to project root
cd ~/Documents/xpi-blocks

# Build and start the container using the simulation config
docker compose -f docker-compose.sim.yml up -d
```

### Enter the Container
```bash
docker exec -it xpi_sim bash
```

---

## 🎮 3. Running the Gazebo Simulation

Once inside the container, initialize the environment and launch the Hexapod simulation.

```bash
# 1. Launch the Hexapod in Gazebo
# This will open the Gazebo 3D environment with the physical model
ros2 launch xpi_projects gazebo.launch.py
```

---

## 🧪 4. Control and Test the Robot

While Gazebo is running, open a **new terminal** on your host and enter the container again to send commands.

### Option A: Run the Automated Test Suite
This is the fastest way to verify everything works (movement, rotation, etc.).
```bash
# Inside the container shell:
bash /Users/slava/Documents/xpi-blocks/projects/hexapod_master/scripts/demos/run_all_tests.sh
```

### Option B: Manual Movement Commands
```bash
# Move forward in the simulator
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Rotate to a specific heading (90 degrees left)
ros2 topic pub --once /hexapod/target_heading std_msgs/msg/Float32 "{data: 1.57}"
```

---

## 💡 Troubleshooting

| Issue | Solution |
| :--- | :--- |
| **"No protocol specified"** | Run `xhost +local:docker` on your **Host** machine. |
| **Gazebo is black or crashes** | Ensure your Host has 3D acceleration enabled. If using NVIDIA, install the `nvidia-container-toolkit`. |
| **I2C/GPIO Errors** | This is expected on PC. Use `mock_hardware:=true` for hardware nodes. |
| **Permission Denied** | Ensure your user is in the `docker` group: `sudo usermod -aG docker $USER`. |

---

## 🎯 Summary of Commands
1.  **Host:** `xhost +local:docker`
2.  **Start:** `docker compose -f docker-compose.sim.yml up -d`
3.  **Simulate:** `docker exec -it xpi_sim ros2 launch xpi_projects gazebo.launch.py`
4.  **Test:** `docker exec -it xpi_sim bash projects/hexapod_master/scripts/demos/run_all_tests.sh`
