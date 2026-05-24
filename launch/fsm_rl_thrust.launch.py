# Copyright 2026 px4_ros2_ctrl contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    thrust_min = LaunchConfiguration("thrust_min")
    thrust_max = LaunchConfiguration("thrust_max")
    thrust_axis = LaunchConfiguration("thrust_axis")
    thrust_sign = LaunchConfiguration("thrust_sign")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Absolute path to the ONNX policy used by rl_thrust_controller",
            ),
            DeclareLaunchArgument("thrust_min", default_value="0.0"),
            DeclareLaunchArgument("thrust_max", default_value="0.9"),
            DeclareLaunchArgument("thrust_axis", default_value="2"),
            DeclareLaunchArgument("thrust_sign", default_value="-1.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            Node(
                package="px4_ros2_ctrl",
                executable="fsm_node",
                name="fsm_node",
                output="screen",
                parameters=[
                    {
                        "active_controller": "rl_thrust",
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
                executable="rl_thrust_controller",
                name="rl_thrust_controller",
                output="screen",
                parameters=[
                    {
                        "model_path": model_path,
                        "thrust_min": ParameterValue(thrust_min, value_type=float),
                        "thrust_max": ParameterValue(thrust_max, value_type=float),
                        "thrust_axis": ParameterValue(thrust_axis, value_type=int),
                        "thrust_sign": ParameterValue(thrust_sign, value_type=float),
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                    }
                ],
            ),
        ]
    )
