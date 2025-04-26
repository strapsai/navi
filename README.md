## Creating a new robot that has Sim2Real capabilities needs: 
Development convention based on [Airstack-Robot](https://github.com/castacks/AirStack/tree/develop/robot). 

1. docker/ # Dockerfile for building the robot image.
2. simulation/ # Simulation files for the robot that talks to isaac sim.
3. ros_ws/ # ROS2 workspace for the robot autonomy and control.

## Autonomy
![img.png](asset/arch.png)