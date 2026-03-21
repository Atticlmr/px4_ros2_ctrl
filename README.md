# PX4-ROS2 bridge

[![GitHub license](https://img.shields.io/github/license/Atticlmr/px4_ros2_ctrl.svg)](https://github.com/Atticlmr/px4_ros2_ctrl/blob/master/LICENSE) [![Build and Test package](https://github.com/Atticlmr/px4_ros2_ctrl/workflows/Build%20and%20Test%20package/badge.svg?branch=master)](https://github.com/Atticlmr/px4_ros2_ctrl/actions)

This package provides example nodes for exchanging data and commands between ROS 2 and PX4.
It also provides a [library](./include/px4_ros2_ctrl/frame_transforms.h) to ease the conversion between ROS 2 and PX4 frame conventions.

## Dependencies

- [`px4_msgs`](https://github.com/PX4/px4_msgs)

## Install, build and usage

Check the [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) and the [ROS 2 Interface](https://docs.px4.io/main/en/ros/ros2_comm.html) sections on the PX4 Devguide for details on how to install the required dependencies, build the package and use it.
