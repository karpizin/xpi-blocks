FROM ros:humble-ros-base

# Prevent prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-gpiozero \
    libgpiod2 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
WORKDIR /xpi_ws

# Copy the entire project
COPY . .

# Install ROS2 dependencies using rosdep
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
# We use a bash shell to source ROS2 environment before building
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add sourcing to bashrc for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /xpi_ws/install/setup.bash" >> ~/.bashrc

# Create a workspace entrypoint script
RUN printf '#!/bin/bash\nset -e\nsource "/opt/ros/humble/setup.bash"\nsource "/xpi_ws/install/setup.bash"\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
