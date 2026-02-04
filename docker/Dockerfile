ARG ROS_DISTRO="jazzy"
# 1. Use the pre-built RMF image as the base
FROM ghcr.io/open-rmf/rmf/rmf_demos:$ROS_DISTRO-rmf-latest AS base
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install tools
RUN apt-get update \
   && apt-get install --no-install-recommends -y  \
   bash-completion \
   curl \
   git \
   python3-colcon-common-extensions \
   python3-pip \
   software-properties-common \
   sudo \
   tmux \
   unzip \
   vim \
   && rm -r /var/lib/apt/lists/*

# 2. Add user `docker`
RUN useradd -m docker -s /bin/bash && echo "docker:docker" | chpasswd \
   && adduser docker sudo \
   && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# --- CRITICAL FIX START ---
# 3. Locate the existing workspace and fix permissions
# We assume the workspace is at /rmf_demos_ws based on the official image structure.
# We change ownership so the 'docker' user can access and use it.
RUN chown -R docker:docker /rmf_demos_ws

USER docker
WORKDIR /home/docker/dev_ws

# 4. Source BOTH the base ROS and the RMF overlay
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/docker/.bashrc \
   && echo "source /rmf_demos_ws/install/setup.bash" >> /home/docker/.bashrc