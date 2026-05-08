// Copyright 2026 px4_ros2_ctrl contributors

#ifndef PX4_ROS2_CTRL__PX4_OUTPUT_ADAPTER_H_
#define PX4_ROS2_CTRL__PX4_OUTPUT_ADAPTER_H_

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "px4_ros2_ctrl/controller_output.h"

namespace px4_ros2_ctrl
{

class Px4OutputAdapter
{
public:
    explicit Px4OutputAdapter(rclcpp::Node *node);

    void publishOffboardHeartbeat(ControlLevel level);
    void publishSetpoint(const ControllerOutput &output);
    void sendArmCommand(bool arm);
    void sendOffboardModeCommand();
    void sendLandCommand();

private:
    rclcpp::Node *node_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

    uint64_t timestampMicros() const;
    void publishVehicleCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f);
};

}  // namespace px4_ros2_ctrl

#endif  // PX4_ROS2_CTRL__PX4_OUTPUT_ADAPTER_H_
