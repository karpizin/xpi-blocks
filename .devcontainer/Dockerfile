FROM ros:humble

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-gpiozero \
    python3-pytest \
    git \
    nano \
    # Tools for clean GPIO access
    libgpiod2 \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if not already done
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi && \
    rosdep update

# Setup workspace
WORKDIR /workspace
COPY . /workspace/

# Build the workspace (Optional, but good check)
# RUN . /opt/ros/humble/setup.sh && colcon build

# Add source to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
