# Use the official ROS 2 Humble base image
FROM osrf/ros:humble-desktop

# Set non-interactive installation to prevent hanging prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies, python tools, and common ROS messages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Set the working workspace inside the container
WORKDIR /workspace

# Copy and install python dependencies first (caches the layer)
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Source the ROS 2 setup script automatically whenever a shell opens
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Expose port 5000 for our FastAPI Web UI
EXPOSE 5000

# Set the default entry command to execute our web server
CMD ["python3", "-m", "uvicorn", "simulator.web_server:app", "--host", "0.0.0.0", "--port", "5000"]