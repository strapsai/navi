import argparse
import logging
from typing import Optional

import rclpy

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from rclpy.node import Node
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import fqn, namespace_with

from spot_msgs.action import RobotCommand  # type: ignore
from geometry_msgs.msg import Pose
from .simple_spot_commander import SimpleSpotCommander
from .robot_commander import RobotCommander

class WalkForward:
    def _process_waypoint(self, waypoint: Pose):
        self._is_moving = True
        self._logger.info(f"Moving to waypoint: {waypoint.position}")

        result = self.consume_waypoint.walk_forward_with_world_frame_goal(waypoint)
        self._logger.info(f"Result: {result}")  
        if result:
            self._logger.info("Waypoint reached.")
        else:
            self._logger.error("Failed to reach waypoint.")

        self._is_moving = False

        # # Check if there are any buffered waypoints and process the next one
        # if self._waypoint_buffer:
        #     next_waypoint = self._waypoint_buffer.pop(0)
        #     self._process_waypoint(next_waypoint)
        
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        self.node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
        self._robot_name = robot_name

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        self._tf_listener = TFListenerWrapper(node)
        self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)
        self._robot = SimpleSpotCommander(robot_name=self._robot_name, node=self.node)
        self.consume_waypoint = RobotCommander(robot_name=self._robot_name, node=self.node)
        self.waypoint_sub = self.node.create_subscription(
            Pose, "/next_waypoint/odom",self.waypoint_callback, 10
        )
        self.waypoint = None
        self._is_moving = False  # Flag to track if the robot is currently moving
        self._waypoint_buffer = []  # Buffer to store incoming waypoints

    def waypoint_callback(self,msg):
        # Check if the robot is currently moving
        if self._is_moving:
            # If the robot is currently moving, buffer the new waypoint
            self._waypoint_buffer.append(msg)
            self._logger.info(f"Buffered waypoint: {msg.position}")
        else:
            # If the robot is not moving, process the waypoint immediately
            self._process_waypoint(msg)

    def initialize_robot(self) -> bool:
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot message was " + result.message)
            return False
        self._logger.info("Claimed robot")

        # Stand the robot up.
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot message was " + result.message)
            return False
        self._logger.info("Standing robot up")
        result = self._robot.command("stand")
        if not result.success:
            self._logger.error("Robot did not stand message was " + result.message)
            return False
        self._logger.info("Successfully stood up.")
        return True
    
    def spin(self):
        rclpy.spin(self.node)



def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    goto = WalkForward(args.robot, main.node)
    goto.initialize_robot()
    goto.spin()
    return 0


if __name__ == "__main__":
    exit(main())
