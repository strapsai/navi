from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    spot_mobility_controller_dir = get_package_share_directory("spot_mobility_controller")
    # Declare launch arguments
    
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="spot", description="Name of the robot"
    )
    namespace = LaunchConfiguration("namespace")

    config_fp_arg = DeclareLaunchArgument(
        "config_file_path",
        default_value=os.path.join(spot_mobility_controller_dir,"config/spot.yaml"),
        description="Path to the configuration file",
    )
    config_fp = LaunchConfiguration("config_file_path")

    #define the node that converts local path to a pose in spot's frame
    odom_controller = Node(
        package="spot_mobility_controller",
        executable="odom_controller",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        name="odom_controller",
        parameters=[
            {
                "config_file_path": config_fp,
                "namespace": namespace
            }
        ],
    )

    # Create the launch description
    return LaunchDescription(
        [
            namespace_arg,
            config_fp_arg,
            odom_controller
        ]
    )