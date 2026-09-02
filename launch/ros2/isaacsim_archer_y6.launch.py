"""Launch the Archer Isaac Sim command forwarder."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Launch the Archer node with its YAML parameters."""
    params = PathJoinSubstitution([
        FindPackageShare("hex_ros_isaacsim_arm"),
        "config", "ros2", "archer_params.yaml"])
    urdf = PathJoinSubstitution([
        FindPackageShare("hex_ros_urdf_archer_y6"), "urdf", "empty.urdf"])
    return LaunchDescription([Node(
        package="hex_ros_isaacsim_arm",
        executable="isaacsim_archer_y6",
        name="hex_ros_isaacsim_archer_y6",
        output="screen",
        parameters=[params, {"urdf_path": urdf}])])
