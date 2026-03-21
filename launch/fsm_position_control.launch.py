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
                'active_controller': 'position'  # 默认使用位置控制
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
