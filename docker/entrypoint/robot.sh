#!/bin/bash

# Source the bashrc first
source ~/.bashrc

# Wait 3 seconds before starting
sleep 3

# Setting home directory
cd ${SIM_DIR}

# Load the tmux session
tmuxp load ${SIM_DIR}/entrypoint/navi-sim.yaml

# Keep container running even if the launch fails
while true; do
    sleep 1
done