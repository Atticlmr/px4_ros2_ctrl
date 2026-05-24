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

#ifndef PX4_ROS2_CTRL__CONTROLLER_OUTPUT_H_
#define PX4_ROS2_CTRL__CONTROLLER_OUTPUT_H_

#include <array>
#include <cstdint>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace px4_ros2_ctrl {

enum class ControlLevel : uint8_t {
  POSITION = 0,
  VELOCITY = 1,
  ACCELERATION = 2,
  ATTITUDE = 3,
  BODY_RATE = 4,
  THRUST = 5,
  ACTUATOR = 6,
};

struct ControllerOutput {
  ControlLevel level = ControlLevel::POSITION;
  rclcpp::Time stamp;
  bool valid = false;
  std::string controller_name;

  std::array<float, 3> position = nanVector();
  std::array<float, 3> velocity = nanVector();
  std::array<float, 3> acceleration = nanVector();
  float yaw = std::numeric_limits<float>::quiet_NaN();
  float yawspeed = std::numeric_limits<float>::quiet_NaN();

  std::array<float, 3> body_rate = nanVector();
  std::array<float, 3> thrust_body = nanVector();
  std::array<float, 3> torque_body = zeroVector();
  bool reset_rate_integral = false;

  static std::array<float, 3> nanVector() {
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    return {nan, nan, nan};
  }

  static std::array<float, 3> zeroVector() { return {0.0f, 0.0f, 0.0f}; }
};

inline const char* toString(ControlLevel level) {
  switch (level) {
  case ControlLevel::POSITION:
    return "position";
  case ControlLevel::VELOCITY:
    return "velocity";
  case ControlLevel::ACCELERATION:
    return "acceleration";
  case ControlLevel::ATTITUDE:
    return "attitude";
  case ControlLevel::BODY_RATE:
    return "body_rate";
  case ControlLevel::THRUST:
    return "thrust";
  case ControlLevel::ACTUATOR:
    return "actuator";
  }
  return "unknown";
}

} // namespace px4_ros2_ctrl

#endif // PX4_ROS2_CTRL__CONTROLLER_OUTPUT_H_
