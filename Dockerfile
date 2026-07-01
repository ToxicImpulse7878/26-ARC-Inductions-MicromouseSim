# syntax=docker/dockerfile:1
# Micromouse Simulator — headless ROS 2 Humble + Pygame, streamed via noVNC.
# Builds cleanly on both linux/amd64 and linux/arm64/v8 (Raspberry Pi / Apple Silicon hosts).

FROM ros:humble-ros-base

LABEL maintainer="ARC Micromouse Induction"
LABEL description="Headless Pygame + ROS 2 Humble micromouse simulator, streamed over noVNC"

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1

# ---------------------------------------------------------------------------
# System dependencies
#   - xvfb        : virtual X11 framebuffer (no physical/host display needed)
#   - x11vnc       : exposes the Xvfb framebuffer over VNC
#   - novnc        : HTML5 VNC client, served over plain HTTP
#   - websockify   : WebSocket<->TCP proxy that noVNC's launch.sh wraps
#   - fluxbox      : minimal window manager so Pygame's window gets decorated
#                     and centered instead of free-floating with no geometry
#   - build-essential / python3-pip : compiling any native Python deps
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    fluxbox \
    x11-xserver-utils \
    build-essential \
    python3-pip \
    python3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# novnc's launch.sh expects "novnc_proxy" / websockify symlinked predictably
# on some distros this is already true, but we pin it explicitly for safety.
RUN ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

# ---------------------------------------------------------------------------
# Python dependencies (pinned for reproducibility across architectures)
# ---------------------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        numpy==1.26.4 \
        pygame==2.5.2

WORKDIR /workspace

# Copy project files. In dev, docker-compose bind-mounts over this anyway,
# but baking it in keeps the image runnable standalone (e.g. `docker run`).
COPY . /workspace

RUN chmod +x /workspace/entrypoint.sh

# noVNC web UI
EXPOSE 8080

# Source the ROS 2 environment for every shell/process in this image.
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT ["bash", "/workspace/entrypoint.sh"]
