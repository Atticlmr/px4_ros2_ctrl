// Copyright 2026 px4_ros2_ctrl contributors

#ifndef PX4_ROS2_CTRL__CONTROLLER_OUTPUT_H_
#define PX4_ROS2_CTRL__CONTROLLER_OUTPUT_H_

#include <array>
#include <cstdint>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace px4_ros2_ctrl
{

enum class ControlLevel : uint8_t
{
    POSITION = 0,
    VELOCITY = 1,
    ACCELERATION = 2,
    ATTITUDE = 3,
    BODY_RATE = 4,
    ACTUATOR = 5,
};

struct ControllerOutput
{
    ControlLevel level = ControlLevel::POSITION;
    rclcpp::Time stamp;
    bool valid = false;
    std::string controller_name;

    std::array<float, 3> position = nanVector();
    std::array<float, 3> velocity = nanVector();
    std::array<float, 3> acceleration = nanVector();
    float yaw = std::numeric_limits<float>::quiet_NaN();
    float yawspeed = std::numeric_limits<float>::quiet_NaN();

    static std::array<float, 3> nanVector()
    {
        const auto nan = std::numeric_limits<float>::quiet_NaN();
        return {nan, nan, nan};
    }
};

inline const char *toString(ControlLevel level)
{
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
    case ControlLevel::ACTUATOR:
        return "actuator";
    }
    return "unknown";
}

}  // namespace px4_ros2_ctrl

#endif  // PX4_ROS2_CTRL__CONTROLLER_OUTPUT_H_
