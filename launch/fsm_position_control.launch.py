from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # FSM转发节点
        Node(
            package='px4_ros2_ctrl',
            executable='fsm_node',
            name='fsm_node',
            output='screen',
            parameters=[{
                'active_controller': 'position',
                'allow_auto_arm': False,
                'land_on_failsafe': False,
                'controller_timeout_s': 0.25,
                'px4_timeout_s': 3.0,
                'estimator_timeout_s': 0.5,
                'offboard_prepare_s': 1.1,
            }]
        ),

        # 位置控制器
        Node(
            package='px4_ros2_ctrl',
            executable='position_controller',
            name='position_controller',
            output='screen'
        ),
    ])
