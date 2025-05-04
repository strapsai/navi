import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Header

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(Pose, '/next_waypoint/odom', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)  # Publish every second

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish_waypoints(self):
        # Store frame names in variables that will be used to
        # compute transformations
        # from_frame_rel = 'sensor'
        # to_frame_rel = 'map'

        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         to_frame_rel,
        #         from_frame_rel,
        #         rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #     return

        # Create a PoseArray message
        # pose_array = PoseArray()
        # pose_array.header.frame_id = 'sensor'
        # pose_array.header.stamp = self.get_clock().now().to_msg()

        # # Define waypoints (example positions and orientations)
        # for i in range(3):
        pose = Pose()
        pose.position.x = -1.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        # pose_array.poses.append(pose)

        self.publisher_.publish(pose)
        # transformed_pose_array = PoseArray()
        # transformed_pose_array.header = Header()
        # transformed_pose_array.header.stamp = self.get_clock().now().to_msg()
        # transformed_pose_array.header.frame_id = 'map'

        # for pose in pose_array.poses:
        #     try:
        #         transform = self.tf_buffer.lookup_transform(
        #             'map', 'sensor', rclpy.time.Time())
        #         transformed_pose = do_transform_pose(pose, transform)
        #         transformed_pose_array.poses.append(transformed_pose)
        #     except Exception as e:
        #         self.get_logger().warn(f"Transform failed: {e}")

        # # # Publish the transformed PoseArray
        # # self.publisher_.publish(transformed_pose_array)
        self.get_logger().info('Published next waypoints')



def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
