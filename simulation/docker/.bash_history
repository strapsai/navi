source ros_ws/install/setup.bash
ros2 launch robot_control robot_control.launch.py
python3 ros_ws/src/quadruped_robot_ROS2/UI/controller.py
ros2 launch robot_control robot_control.launch.py
cd ros_ws/
colcon build --symlink-install
source ros_ws/install/setup.bash
source install/setup.bash
ros2 launch isaacsim run_isaacsim.launch.py \
  install_path:="${ISAAC_PATH:-/isaac-sim}" \
  gui:="${SIM_DIR:-/isaac-sim}/assets/${DEFAULT_ISAAC_SCENE_FILENAME:-spot_and_arm.robot.usd}" \
  play_sim_on_start:="${PLAY_SIM_ON_START:-true}"
ros2 launch isaacsim run_isaacsim.launch.py install_path:=/isaac-sim gui:=${DEFAULT_ISAAC_SCENE} play_sim_on_start:=${PLAY_SIM_ON_START}\"
exit
ros2 launch isaacsim run_isaacsim.launch.py install_path:=/isaac-sim gui:=${DEFAULT_ISAAC_SCENE} play_sim_on_start:=${PLAY_SIM_ON_START}\"
SCENE_FILE="${SIM_DIR}/assets/${DEFAULT_ISAAC_SCENE_FILENAME}"
if [ -f "$SCENE_FILE" ]; then     echo "Scene file found at: $SCENE_FILE"; else     echo "ERROR: Scene file not found at: $SCENE_FILE";     exit 1; fi
