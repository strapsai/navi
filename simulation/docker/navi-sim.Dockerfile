# Dockerfile for Rayfronts

ARG BASE_IMAGE
FROM ${BASE_IMAGE}


# Install additional dependencies that aren't in the base image
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    htop \
    tmux \
    wget \
    git \
    libglfw3-dev \
    python3-pip \
    python3-dev \
    python3-setuptools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*




RUN  apt-get clean \
&& rm -rf /var/lib/apt/lists/*