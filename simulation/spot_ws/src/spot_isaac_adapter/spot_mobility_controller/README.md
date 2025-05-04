# Spot Mobility Controller

## Overview
`spot_mobility_controller` is a ROS 2 package designed to control the movement of Boston Dynamics' Spot robot using waypoints as a topic. The package subscribes to pose messages from the `/next_waypoint/odom` topic and directs Spot to move towards the received waypoints while ensuring smooth execution using ros2 actions.

## Features
- Initializes and claims control over Spot.
- Powers on and makes Spot stand.
- Receives waypoint data from the `/next_waypoint/odom` topic.
- Transforms waypoints into the world frame and executes movement commands.
- Ensures continuous navigation by buffering waypoints when Spot is busy.


## Usage
### Running the Controller
To start the Spot mobility controller, run:
```sh
ros2 run spot_mobility_controller mobility_controller
```

### Launching with ROS 2(TODO)
You can create a launch file or directly launch using:
```sh
ros2 launch spot_mobility_controller mobility_controller.launch.py
```

## Code Structure
### `robot_commander.py`
This module defines the `RobotCommander` class, which provides functions for:
- Initializing Spot
- Handling movement commands
- Transforming waypoints to the world frame

### `mobility_controller.py`
The main entry point for the package, responsible for:
- Subscribing to `/next_waypoint/odom`
- Executing movement commands
- Managing waypoint buffering

## Todo
Create a launch file 


## Contributors
For inquiries or contributions, contact Kabir Kedia(kabirk@andrew.cmu.edu)

