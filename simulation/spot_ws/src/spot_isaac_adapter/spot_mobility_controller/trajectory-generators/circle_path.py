import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(Pose, '/next_waypoint/odom', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)  # Publish every second
        
        self.radius = 0.6  # Radius of the circle
        self.angle_deg = 0.0   # Initial angle in degrees
        self.angular_step_deg = 10  # Step in degrees

    def publish_waypoints(self):
        angle_rad = math.radians(self.angle_deg)
        pose = Pose()
        pose.position.x = self.radius * math.cos(angle_rad)
        pose.position.y = self.radius * math.sin(angle_rad)
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(angle_rad / 2)
        pose.orientation.w = math.cos(angle_rad / 2)

        self.publisher_.publish(pose)
        self.get_logger().info(f'Published waypoint: x={pose.position.x}, y={pose.position.y}, angle={self.angle_deg} degrees')

        self.angle_deg += self.angular_step_deg
        if self.angle_deg >= 360.0:
            self.angle_deg -= 360.0  # Keep angle within [0, 360]


def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
