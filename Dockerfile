# Stage with only the package manifests: its content changes only when a package.xml
# changes, so the rosdep layer below stays cached when only code changes.
FROM ros:humble-ros-base AS manifests
COPY src /src
RUN find /src -type f ! -name package.xml -delete && find /src -type d -empty -delete

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
# Known, checked conflict: mediapipe declares protobuf<5, while meshtastic needs protobuf 5.
# mp.solutions.hands (gesture_control) runs fine with protobuf 5; forcing protobuf<5 would
# make pip downgrade meshtastic to 2.3 and mediapipe to 0.10.9, so the conflict is accepted.
# Recheck it when changing any of these packages.
# google-genai is the Gemini SDK (google-generativeai is deprecated).
RUN pip3 install --no-cache-dir --upgrade pip wheel "setuptools<80" && \
    pip3 install --no-cache-dir -r requirements.txt && \
    pip3 install --no-cache-dir \
    openai \
    google-genai

# Setup workspace
WORKDIR /xpi_ws

# Install ROS2 dependencies using rosdep (from the package manifests only, see above)
COPY --from=manifests /src ./src
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Copy the entire project
COPY . .

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
