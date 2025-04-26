#!/bin/bash

# Source the bashrc first
source ~/.bashrc

# Wait 3 seconds before starting
sleep 3

# Setting home directory
cd ${NAVI_DIR}

# Load the tmux session
tmuxp load ${NAVI_DIR}/entrypoint/navi.yaml

# Keep container running even if the launch fails
while true; do
    sleep 1
done