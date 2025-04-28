source ros_ws/install/setup.bash
ros2 launch robot_control robot_control.launch.py
python3 ros_ws/src/quadruped_robot_ROS2/UI/controller.py
clear
ls
cd ros_ws/
ls
cd src/
cd
cd $SIM_DIR/
ls
clear
ls
ros2 launch robot_control robot_control.launch.py
cd ros_ws/
colcon build --symlink-install
source ros_ws/install/setup.bash
source install/setup.bash
ros2 launch robot_control robot_control.launch.py
clear
exit
