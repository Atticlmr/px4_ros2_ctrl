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

#ifndef PX4_ROS2_CTRL__FSM_NODE_H_
#define PX4_ROS2_CTRL__FSM_NODE_H_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include "px4_ros2_ctrl/controller_output.h"
#include "px4_ros2_ctrl/log_style.h"
#include "px4_ros2_ctrl/px4_output_adapter.h"

namespace px4_ros2_ctrl {

class FSMNode : public rclcpp::Node {
public:
  FSMNode();

private:
  enum class State {
    WAIT_FOR_PX4,
    STANDBY,
    OFFBOARD_PREPARE,
    OFFBOARD_REQUESTED,
    OFFBOARD_ACTIVE,
    MANUAL_OVERRIDE,
    FAILSAFE,
  };

  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr position_output_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr body_rate_output_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_output_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_offboard_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_offboard_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_override_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<Px4OutputAdapter> px4_adapter_;
  ControllerOutput controller_output_;

  px4_msgs::msg::VehicleControlMode current_control_mode_;
  px4_msgs::msg::VehicleLocalPosition current_local_position_;

  State state_ = State::WAIT_FOR_PX4;
  State previous_state_ = State::WAIT_FOR_PX4;

  bool has_control_mode_ = false;
  bool has_local_position_ = false;
  bool has_controller_output_ = false;
  bool has_start_request_ = false;
  bool manual_override_latched_ = false;
  bool arm_on_start_ = false;
  bool land_on_failsafe_ = false;
  bool color_logs_ = false;

  rclcpp::Time last_control_mode_time_;
  rclcpp::Time last_local_position_time_;
  rclcpp::Time last_controller_output_time_;
  rclcpp::Time state_enter_time_;

  std::string active_controller_;
  ControlLevel active_control_level_ = ControlLevel::POSITION;
  double px4_timeout_s_ = 3.0;
  double estimator_timeout_s_ = 0.5;
  double controller_timeout_s_ = 0.25;
  double offboard_prepare_s_ = 1.1;
  bool allow_auto_arm_ = false;

  void controlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void manualControlCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg);
  void positionOutputCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);
  void bodyRateOutputCallback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg);
  void thrustOutputCallback(const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg);

  void startOffboardCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stopOffboardCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void resetOverrideCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void controlLoop();
  void transitionTo(State next_state, const std::string& reason);
  bool px4ConnectionHealthy() const;
  bool estimatorHealthy() const;
  bool controllerOutputFresh() const;
  bool px4InOffboard() const;
  bool px4InManualMode() const;
  bool px4Armed() const;
  bool offboardState() const;
  log_style::Color stateLogColor(State state) const;
  const char* stateName(State state) const;
};

} // namespace px4_ros2_ctrl

#endif // PX4_ROS2_CTRL__FSM_NODE_H_
