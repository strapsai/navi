import math
import sys
import yaml

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('static_turtle_tf2_broadcaster')
        self.declare_parameter("config_file_path","")
        self.config_file = self.get_parameter("config_file_path").get_parameter_value().string_value

        with open(self.config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        
        self.translation = self.config['transform_params']['translation']
        self.quaternion = self.config['transform_params']['quaternion']

        self.from_frame = self.config['transform_params']['from_frame']
        self.to_frame = self.config['transform_params']['to_frame']

        self.debug = self.config['debug']['enabled']
        if self.debug:
            self.get_logger().info(f"From Frame: {self.from_frame}")
            self.get_logger().info(f"To Frame: {self.to_frame}")
            self.get_logger().info(f"Translation: {self.translation}")
            self.get_logger().info(f"Quaternion: {self.quaternion}")

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.from_frame
        t.child_frame_id = self.to_frame

        t.transform.translation.x = float(self.translation[0])
        t.transform.translation.y = float(self.translation[1])
        t.transform.translation.z = float(self.translation[2])

        t.transform.rotation.x = self.quaternion[0]
        t.transform.rotation.y = self.quaternion[1]
        t.transform.rotation.z = self.quaternion[2]
        t.transform.rotation.w = self.quaternion[3]

        self.tf_static_broadcaster.sendTransform(t)


def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()