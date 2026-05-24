// Copyright 2026 px4_ros2_ctrl contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "px4_ros2_ctrl/px4_output_adapter.h"

namespace px4_ros2_ctrl {

Px4OutputAdapter::Px4OutputAdapter(rclcpp::Node* node) : node_(node) {
  offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);
  trajectory_pub_ =
    node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
  rates_pub_ = node_->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
    "/fmu/in/vehicle_rates_setpoint", 10);
  thrust_pub_ = node_->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
    "/fmu/in/vehicle_thrust_setpoint", 10);
  torque_pub_ = node_->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
    "/fmu/in/vehicle_torque_setpoint", 10);
  vehicle_cmd_pub_ =
    node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
}

void Px4OutputAdapter::publishOffboardHeartbeat(ControlLevel level) {
  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = timestampMicros();
  msg.position = level == ControlLevel::POSITION;
  msg.velocity = level == ControlLevel::VELOCITY;
  msg.acceleration = level == ControlLevel::ACCELERATION;
  msg.attitude = level == ControlLevel::ATTITUDE;
  msg.body_rate = level == ControlLevel::BODY_RATE;
  msg.thrust_and_torque = level == ControlLevel::THRUST;
  msg.direct_actuator = level == ControlLevel::ACTUATOR;
  offboard_pub_->publish(msg);
}

void Px4OutputAdapter::publishSetpoint(const ControllerOutput& output) {
  if (!output.valid) { return; }

  if (output.level == ControlLevel::POSITION || output.level == ControlLevel::VELOCITY ||
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

  if (output.level == ControlLevel::BODY_RATE) {
    px4_msgs::msg::VehicleRatesSetpoint msg;
    msg.timestamp = timestampMicros();
    msg.roll = output.body_rate[0];
    msg.pitch = output.body_rate[1];
    msg.yaw = output.body_rate[2];
    msg.thrust_body = output.thrust_body;
    msg.reset_integral = output.reset_rate_integral;
    rates_pub_->publish(msg);
    return;
  }

  if (output.level == ControlLevel::THRUST) {
    const auto timestamp = timestampMicros();

    px4_msgs::msg::VehicleThrustSetpoint thrust_msg;
    thrust_msg.timestamp = timestamp;
    thrust_msg.timestamp_sample = timestamp;
    thrust_msg.xyz = output.thrust_body;
    thrust_pub_->publish(thrust_msg);

    px4_msgs::msg::VehicleTorqueSetpoint torque_msg;
    torque_msg.timestamp = timestamp;
    torque_msg.timestamp_sample = timestamp;
    torque_msg.xyz = output.torque_body;
    torque_pub_->publish(torque_msg);
    return;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "Controller output level '%s' has no PX4 adapter implementation yet",
                       toString(output.level));
}

void Px4OutputAdapter::sendArmCommand(bool arm) {
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        arm ? 1.0f : 0.0f);
}

void Px4OutputAdapter::sendOffboardModeCommand() {
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void Px4OutputAdapter::sendLandCommand() {
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

uint64_t Px4OutputAdapter::timestampMicros() const {
  return node_->get_clock()->now().nanoseconds() / 1000;
}

void Px4OutputAdapter::publishVehicleCommand(uint32_t command, float param1, float param2) {
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

} // namespace px4_ros2_ctrl
