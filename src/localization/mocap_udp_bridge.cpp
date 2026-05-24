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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "px4_ros2_ctrl/third_party/nlohmann/json.hpp"

namespace {

using json = nlohmann::json;

constexpr auto kNan = std::numeric_limits<float>::quiet_NaN();

std::array<double, 3> readVector3(const json& data, const char* field,
                                  const std::array<double, 3>& default_value) {
  if (!data.contains(field) || data.at(field).is_null()) { return default_value; }

  const auto& value = data.at(field);
  if (!value.is_array() || value.size() != 3) {
    throw std::runtime_error(std::string(field) + " must be an array with 3 numbers");
  }

  return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}

std::array<double, 4> readQuaternionWxyz(const json& data, const char* field,
                                         const std::array<double, 4>& default_value) {
  if (!data.contains(field) || data.at(field).is_null()) { return default_value; }

  const auto& value = data.at(field);
  if (!value.is_array() || value.size() != 4) {
    throw std::runtime_error(std::string(field) + " must be an array with 4 numbers");
  }

  return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>(),
          value.at(3).get<double>()};
}

bool finiteVector(const std::array<double, 3>& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

Eigen::Vector3d enuToNed(const Eigen::Vector3d& value) {
  return Eigen::Vector3d(value.y(), value.x(), -value.z());
}

Eigen::Quaterniond enuToNed(const Eigen::Quaterniond& value) {
  const Eigen::Quaterniond enu_to_ned = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  return enu_to_ned * value;
}

} // namespace

namespace px4_ros2_ctrl {

class MocapUdpBridge : public rclcpp::Node {
public:
  MocapUdpBridge() : Node("mocap_udp_bridge") {
    declare_parameter<std::string>("bind_ip", "0.0.0.0");
    declare_parameter<int>("bind_port", 5005);
    declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_visual_odometry");
    declare_parameter<int>("max_packet_size", 4096);
    declare_parameter<double>("position_variance", 0.0025);
    declare_parameter<double>("orientation_variance", 0.0004);
    declare_parameter<double>("velocity_variance", 0.01);
    declare_parameter<bool>("input_frame_enu", true);

    bind_ip_ = get_parameter("bind_ip").as_string();
    bind_port_ = get_parameter("bind_port").as_int();
    output_topic_ = get_parameter("output_topic").as_string();
    max_packet_size_ = get_parameter("max_packet_size").as_int();
    position_variance_ = static_cast<float>(get_parameter("position_variance").as_double());
    orientation_variance_ = static_cast<float>(get_parameter("orientation_variance").as_double());
    velocity_variance_ = static_cast<float>(get_parameter("velocity_variance").as_double());
    input_frame_enu_ = get_parameter("input_frame_enu").as_bool();

    if (bind_port_ <= 0 || bind_port_ > 65535) {
      throw std::runtime_error("bind_port must be in range 1..65535");
    }
    if (max_packet_size_ <= 0) { throw std::runtime_error("max_packet_size must be positive"); }

    odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(output_topic_, 10);

    openSocket();
    running_ = true;
    receive_thread_ = std::thread(&MocapUdpBridge::receiveLoop, this);

    RCLCPP_INFO(get_logger(), "Mocap UDP bridge listening on %s:%d, publishing %s",
                bind_ip_.c_str(), bind_port_, output_topic_.c_str());
  }

  ~MocapUdpBridge() override {
    running_ = false;
    if (socket_fd_ >= 0) {
      shutdown(socket_fd_, SHUT_RDWR);
      close(socket_fd_);
      socket_fd_ = -1;
    }
    if (receive_thread_.joinable()) { receive_thread_.join(); }
  }

private:
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
  std::thread receive_thread_;
  std::atomic_bool running_{false};

  std::string bind_ip_;
  std::string output_topic_;
  int bind_port_ = 5005;
  int max_packet_size_ = 4096;
  int socket_fd_ = -1;
  float position_variance_ = 0.0025f;
  float orientation_variance_ = 0.0004f;
  float velocity_variance_ = 0.01f;
  bool input_frame_enu_ = true;

  void openSocket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error(std::string("failed to create UDP socket: ") + std::strerror(errno));
    }

    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(bind_port_));
    if (inet_pton(AF_INET, bind_ip_.c_str(), &address.sin_addr) != 1) {
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("bind_ip must be a valid IPv4 address");
    }

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error(std::string("failed to bind UDP socket: ") + std::strerror(errno));
    }
  }

  void receiveLoop() {
    std::string packet;
    packet.resize(static_cast<size_t>(max_packet_size_));

    while (running_) {
      const ssize_t received = recv(socket_fd_, packet.data(), packet.size() - 1, 0);
      if (received <= 0) {
        if (running_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "UDP receive failed: %s",
                               std::strerror(errno));
        }
        continue;
      }

      try {
        publishOdometryFromJson(std::string(packet.data(), static_cast<size_t>(received)));
      } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Dropped mocap UDP packet: %s",
                             error.what());
      }
    }
  }

  void publishOdometryFromJson(const std::string& packet) {
    const auto data = json::parse(packet);

    const auto position = readVector3(data, "position", {kNan, kNan, kNan});
    const auto velocity = readVector3(data, "velocity", {kNan, kNan, kNan});
    const auto quaternion = readQuaternionWxyz(data, "quaternion", {1.0, 0.0, 0.0, 0.0});

    if (!finiteVector(position)) {
      throw std::runtime_error("position must contain finite numbers");
    }

    const auto source_timestamp_us = data.value<uint64_t>("timestamp_us", nowMicroseconds());

    Eigen::Vector3d position_vector(position[0], position[1], position[2]);
    Eigen::Vector3d velocity_vector(velocity[0], velocity[1], velocity[2]);
    Eigen::Quaterniond orientation(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

    if (!std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
      throw std::runtime_error("quaternion norm is invalid");
    }
    orientation.normalize();

    if (input_frame_enu_) {
      position_vector = enuToNed(position_vector);
      if (finiteVector(velocity)) { velocity_vector = enuToNed(velocity_vector); }
      orientation = enuToNed(orientation);
      orientation.normalize();
    }

    px4_msgs::msg::VehicleOdometry msg;
    msg.timestamp = nowMicroseconds();
    msg.timestamp_sample = source_timestamp_us;
    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    msg.position = {static_cast<float>(position_vector.x()),
                    static_cast<float>(position_vector.y()),
                    static_cast<float>(position_vector.z())};

    msg.q = {static_cast<float>(orientation.w()), static_cast<float>(orientation.x()),
             static_cast<float>(orientation.y()), static_cast<float>(orientation.z())};

    msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    if (finiteVector(velocity)) {
      msg.velocity = {static_cast<float>(velocity_vector.x()),
                      static_cast<float>(velocity_vector.y()),
                      static_cast<float>(velocity_vector.z())};
    } else {
      msg.velocity = {kNan, kNan, kNan};
    }

    msg.angular_velocity = {kNan, kNan, kNan};
    msg.position_variance = {position_variance_, position_variance_, position_variance_};
    msg.orientation_variance = {orientation_variance_, orientation_variance_,
                                orientation_variance_};
    msg.velocity_variance = {velocity_variance_, velocity_variance_, velocity_variance_};
    msg.reset_counter = 0;
    msg.quality = 100;

    odom_pub_->publish(msg);
  }

  uint64_t nowMicroseconds() const { return static_cast<uint64_t>(now().nanoseconds() / 1000); }
};

} // namespace px4_ros2_ctrl

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_ros2_ctrl::MocapUdpBridge>());
  rclcpp::shutdown();
  return 0;
}
