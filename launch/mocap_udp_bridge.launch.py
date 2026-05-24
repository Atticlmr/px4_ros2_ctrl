from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bind_ip = LaunchConfiguration("bind_ip")
    bind_port = LaunchConfiguration("bind_port")
    output_topic = LaunchConfiguration("output_topic")
    input_frame_enu = LaunchConfiguration("input_frame_enu")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("bind_port", default_value="5005"),
            DeclareLaunchArgument("output_topic", default_value="/fmu/in/vehicle_visual_odometry"),
            DeclareLaunchArgument("input_frame_enu", default_value="true"),
            Node(
                package="px4_ros2_ctrl",
                executable="mocap_udp_bridge",
                name="mocap_udp_bridge",
                output="screen",
                parameters=[
                    {
                        "bind_ip": bind_ip,
                        "bind_port": bind_port,
                        "output_topic": output_topic,
                        "input_frame_enu": input_frame_enu,
                    }
                ],
            ),
        ]
    )
