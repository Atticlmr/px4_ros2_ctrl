// Copyright 2026 px4_ros2_ctrl contributors

#include "px4_ros2_ctrl/px4_output_adapter.h"

namespace px4_ros2_ctrl
{

Px4OutputAdapter::Px4OutputAdapter(rclcpp::Node *node) : node_(node)
{
    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    trajectory_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    vehicle_cmd_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
}

void Px4OutputAdapter::publishOffboardHeartbeat(ControlLevel level)
{
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = timestampMicros();
    msg.position = level == ControlLevel::POSITION;
    msg.velocity = level == ControlLevel::VELOCITY;
    msg.acceleration = level == ControlLevel::ACCELERATION;
    msg.attitude = level == ControlLevel::ATTITUDE;
    msg.body_rate = level == ControlLevel::BODY_RATE;
    msg.thrust_and_torque = false;
    msg.direct_actuator = level == ControlLevel::ACTUATOR;
    offboard_pub_->publish(msg);
}

void Px4OutputAdapter::publishSetpoint(const ControllerOutput &output)
{
    if (!output.valid) {
        return;
    }

    if (output.level == ControlLevel::POSITION ||
        output.level == ControlLevel::VELOCITY ||
        output.level == ControlLevel::ACCELERATION) {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = timestampMicros();
        msg.position = output.position;
        msg.velocity = output.velocity;
        msg.acceleration = output.acceleration;
        msg.yaw = output.yaw;
        msg.yawspeed = output.yawspeed;
        trajectory_pub_->publish(msg);
        return;
    }

    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Controller output level '%s' has no PX4 adapter implementation yet",
        toString(output.level));
}

void Px4OutputAdapter::sendArmCommand(bool arm)
{
    publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        arm ? 1.0f : 0.0f);
}

void Px4OutputAdapter::sendOffboardModeCommand()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void Px4OutputAdapter::sendLandCommand()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

uint64_t Px4OutputAdapter::timestampMicros() const
{
    return node_->get_clock()->now().nanoseconds() / 1000;
}

void Px4OutputAdapter::publishVehicleCommand(uint32_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestampMicros();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_cmd_pub_->publish(msg);
}

}  // namespace px4_ros2_ctrl
