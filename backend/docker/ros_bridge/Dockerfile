FROM osrf/ros:foxy-desktop

# Install ROS 1 Noetic and ros1_bridge for Foxy
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-noetic-desktop-full \
      ros-foxy-ros1-bridge \
      python3-pip \
      bash-completion \
      curl \
      vim \
      && rm -rf /var/lib/apt/lists/*

# Create workspace directory for mounting project code
WORKDIR /workspace

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Default environment variables (can be overridden by docker-compose or -e)
ENV ROS_DISTRO_ROS2=foxy \
    ROS_DISTRO_ROS1=noetic \
    ROS_MASTER_URI=http://127.0.0.1:11311 \
    ROS_IP=127.0.0.1 \
    ROS_DOMAIN_ID=0

ENTRYPOINT ["/entrypoint.sh"]

