# This file interfaces with spot to send action. Helper files are robot_commander and simple_spot_commander

import yaml
import os
import argparse
from typing import List, Optional
from contextlib import closing
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray

from synchros2.subscription import Subscription
import synchros2.process as ros_process
from synchros2.utilities import namespace_with
from .robot_commander import RobotCommander
from ament_index_python.packages import get_package_share_directory

class RobotController:
    def __init__(self, robot_name: Optional[str] = None):
        if robot_name is None:
            self.robot_name = os.getenv("SPOT_NAME")  # Get robot name from environment variable
        else:
            self.robot_name = robot_name
        self.robot_commander = RobotCommander(robot_name=self.robot_name)
        if not self.robot_commander:
            raise ValueError("Failed to create RobotCommander instance.")
        self.robot_commander._node.declare_parameter("config_file_path", "")
        self.config_file_path = self.robot_commander._node.get_parameter("config_file_path").get_parameter_value().string_value

        self.config = self.load_config()
        self.is_busy = False
        self.latest_message = None
    def load_config(self):
        try:
            with open(self.config_file_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            self.robot_commander._logger.error(f"Failed to load config file: {e}")
            return {}

    def initialize_robot(self):
        self.robot_commander._logger.info(f"Initializing {self.robot_commander._robot_name}")
        if not self.robot_commander.initialize_robot():
            self.robot_commander._logger.error("Failed to initialize robot")
            return False
        self.robot_commander._logger.info("Robot initialized successfully")
        return True

    def process_message(self, message):
        if self.is_busy:
            self.latest_message = message
            self.robot_commander._logger.info("Robot is busy, buffering the latest message")
            return
        
        self.execute_walk(message)
        
        # if self.latest_message:
        #     self.execute_walk(self.latest_message)
        #     self.latest_message = None

    def execute_walk(self, message):
        self.is_busy = True
        self.robot_commander.walk_forward_with_world_frame_goal(message)
        self.is_busy = False

    def start(self):
        if not self.initialize_robot():
            return
        
        spot_controller_topic = self.config.get('topic_name', {}).get('spot_controller', '').lower()
        spot_controller_topic = namespace_with(self.robot_name, spot_controller_topic)
        debug = self.config.get('debug', {}).get('enabled', False)
        
        if debug:
            self.robot_commander._logger.info(f"Spot Controller Topic: {spot_controller_topic}")
        
        topic_data = Subscription(Pose, spot_controller_topic)
        
        with closing(topic_data.stream()) as stream:
            for message in stream:
                self.process_message(message)

@ros_process.main()
def main():
    parser = argparse.ArgumentParser(description='Spot Robot Controller')
    parser.add_argument('--robot_name', type=str, help='Name of the robot')
    args, _ = parser.parse_known_args()
    controller = RobotController(robot_name=args.robot_name)
    controller.start()

if __name__ == "__main__":
    main()

