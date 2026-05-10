from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    solver_json = LaunchConfiguration("solver_json")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "solver_json",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("px4_ros2_ctrl"), "generated", "body_rate_nmpc", "body_rate_nmpc.json"]
                ),
                description="Optional absolute path to generated/body_rate_nmpc/body_rate_nmpc.json",
            ),
            Node(
                package="px4_ros2_ctrl",
                executable="fsm_node",
                name="fsm_node",
                output="screen",
                parameters=[
                    {
                        "active_controller": "body_rate_nmpc",
                        "allow_auto_arm": False,
                        "land_on_failsafe": False,
                        "controller_timeout_s": 0.25,
                        "px4_timeout_s": 3.0,
                        "estimator_timeout_s": 0.5,
                        "offboard_prepare_s": 1.1,
                    }
                ],
            ),
            Node(
                package="px4_ros2_ctrl",
                executable="body_rate_nmpc_controller.py",
                name="body_rate_nmpc_controller",
                output="screen",
                parameters=[{"solver_json": solver_json}],
            ),
        ]
    )
