import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

# Get robot name from environment variable
ROBOT_NAME = os.getenv("ROBOT_NAME")


class PoseArrayProcessor(Node):
    def __init__(self):
        super().__init__('pose_array_processor')

        # Load configuration
        self.config = self.load_config()

        # Setup parameters from config
        self.setup_parameters()

        # Create ROS interfaces
        self.setup_ros_interfaces()

        # Setup TF listener
        self.setup_tf()

    def load_config(self):
        """Load the configuration YAML file."""
        self.declare_parameter("config_file_path", "")
        config_path = self.get_parameter("config_file_path").get_parameter_value().string_value
        with open(config_path, 'r') as file:
            return yaml.safe_load(file)

    def setup_parameters(self):
        """Extract topic names and debug flag from config."""
        self.local_planner_topic = self.config['topic_name']['local_planner'].lower()
        self.spot_controller_topic = self.config['topic_name']['spot_controller'].lower()
        self.debug = self.config['debug']['enabled']

        if self.debug:
            self.get_logger().info(f"Local Planner Topic: {self.local_planner_topic}")
            self.get_logger().info(f"Spot Controller Topic: {self.spot_controller_topic}")

    def setup_ros_interfaces(self):
        """Initialize publisher and subscriber."""
        self.subscription = self.create_subscription(
            Path,
            self.local_planner_topic,
            self.path_callback,
            10
        )
        self.publisher = self.create_publisher(Pose, self.spot_controller_topic, 10)

    def setup_tf(self):
        """Setup TF2 buffer and listener."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.to_frame = ROBOT_NAME + self.config['transform_params']['to_frame']

    def select_pose_from_path(self, msg: Path) -> PoseStamped:
        """
        Select a pose from the Path message.
        Picks the 6th pose if available; otherwise returns the last pose.
        """
        idx = min(5, len(msg.poses) - 1)
        return msg.poses[idx]
    
    def validate_timestamps(self, msg: Path) -> bool:
        """
        Ensure all poses in the path share the same timestamp as the Path header.
        """
        path_stamp = msg.header.frame_id
        for i, pose_stamped in enumerate(msg.poses):
            if pose_stamped.header.frame_id != path_stamp:
                self.get_logger().error(
                    f"Timestamp mismatch at index {i}: "
                    f"pose_stamp={pose_stamped.header.stamp}, path_stamp={path_stamp}"
                )
                return False
        return True
    
    def path_callback(self, msg: Path):
        """
        Callback function for Path messages.
        Selects a pose, transforms it, and publishes it.
        """
        if not msg.poses:
            self.get_logger().warn('Received empty Path message')
            return
        if not self.validate_timestamps(msg):
            self.get_logger().error('Timestamps in Path do not match. Ignoring message.')
            return

        selected_pose_stamped = self.select_pose_from_path(msg)

        self.from_frame = selected_pose_stamped.header.frame_id

        if self.debug:
            self.get_logger().info(f"Selected pose: {selected_pose_stamped}")
            self.get_logger().info(f"From frame: {self.from_frame}")
            self.get_logger().info(f"To frame: {self.to_frame}")

        try:
            transform = self.tf_buffer.lookup_transform(
                self.to_frame,
                self.from_frame,
                rclpy.time.Time()
                # msg.header.stamp
            )
            transformed_pose = do_transform_pose(selected_pose_stamped, transform)
            self.publisher.publish(transformed_pose.pose)
            self.get_logger().info('Published transformed pose')
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.from_frame} to {self.to_frame}: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
