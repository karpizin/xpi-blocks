FROM ros:humble-ros-base

# Prevent prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-gpiozero \
    libgpiod2 \
    curl \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

# Install LLM and project dependencies via pip
COPY requirements.txt .
# setuptools<80: colcon-core (ROS 2 Humble) requires it; a newer one is a broken dependency.
# Known, checked conflict: mediapipe declares protobuf<5, while meshtastic and the Google
# client libraries need protobuf 5. mp.solutions.hands (gesture_control) runs fine with
# protobuf 5; forcing protobuf<5 would make pip downgrade meshtastic to 2.3 and mediapipe
# to 0.10.9, so the conflict is accepted. Recheck it when changing any of these packages.
RUN pip3 install --no-cache-dir --upgrade pip wheel "setuptools<80" && \
    pip3 install --no-cache-dir -r requirements.txt && \
    pip3 install --no-cache-dir \
    openai \
    google-generativeai

# Setup workspace
WORKDIR /xpi_ws

# Copy the entire project
COPY . .

# Install ROS2 dependencies using rosdep
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rm -rf build install log && colcon build --merge-install --event-handlers console_direct+"

# Add sourcing to bashrc for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /xpi_ws/install/setup.bash" >> ~/.bashrc

# Create a workspace entrypoint script
RUN printf '#!/bin/bash\nset -e\nsource "/opt/ros/humble/setup.bash"\nsource "/xpi_ws/install/setup.bash"\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
