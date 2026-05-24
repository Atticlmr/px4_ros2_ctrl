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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

using namespace std::chrono_literals;

namespace {

bool finite(float value) {
  return std::isfinite(value);
}

float clamp(float value, float min_value, float max_value) {
  return std::min(std::max(value, min_value), max_value);
}

} // namespace

class RlThrustController : public rclcpp::Node {
public:
  RlThrustController()
    : Node("rl_thrust_controller"), env_(ORT_LOGGING_LEVEL_WARNING, "rl_thrust_controller") {
    declare_parameter<std::string>("model_path", "");
    declare_parameter<std::string>("input_name", "");
    declare_parameter<std::string>("output_name", "");
    declare_parameter<int>("observation_size", 10);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<double>("thrust_min", 0.0);
    declare_parameter<double>("thrust_max", 0.9);
    declare_parameter<double>("thrust_sign", -1.0);
    declare_parameter<int>("thrust_axis", 2);
    declare_parameter<bool>("hold_last_on_inference_error", false);

    model_path_ = get_parameter("model_path").as_string();
    input_name_ = get_parameter("input_name").as_string();
    output_name_ = get_parameter("output_name").as_string();
    observation_size_ = get_parameter("observation_size").as_int();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    thrust_min_ = static_cast<float>(get_parameter("thrust_min").as_double());
    thrust_max_ = static_cast<float>(get_parameter("thrust_max").as_double());
    thrust_sign_ = static_cast<float>(get_parameter("thrust_sign").as_double());
    thrust_axis_ = get_parameter("thrust_axis").as_int();
    hold_last_on_inference_error_ = get_parameter("hold_last_on_inference_error").as_bool();

    if (observation_size_ <= 0) { throw std::runtime_error("observation_size must be positive"); }
    if (publish_rate_hz_ <= 0.0) { throw std::runtime_error("publish_rate_hz must be positive"); }
    if (thrust_axis_ < 0 || thrust_axis_ > 2) {
      throw std::runtime_error("thrust_axis must be 0, 1, or 2");
    }
    if (thrust_min_ > thrust_max_) { throw std::runtime_error("thrust_min must be <= thrust_max"); }

    observation_.assign(static_cast<size_t>(observation_size_), 0.0f);
    last_thrust_body_ = {0.0f, 0.0f, 0.0f};

    thrust_pub_ =
      create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("/controller/thrust/output", 10);

    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
      std::bind(&RlThrustController::odometryCallback, this, std::placeholders::_1));

    loadModel();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&RlThrustController::timerCallback, this));

    RCLCPP_INFO(get_logger(), "RL thrust controller started");
    RCLCPP_INFO(get_logger(), "Publishing THRUST output to /controller/thrust/output");
  }

private:
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;

  std::string model_path_;
  std::string input_name_;
  std::string output_name_;
  int observation_size_ = 10;
  double publish_rate_hz_ = 50.0;
  float thrust_min_ = 0.0f;
  float thrust_max_ = 0.9f;
  float thrust_sign_ = -1.0f;
  int thrust_axis_ = 2;
  bool hold_last_on_inference_error_ = false;
  bool model_ready_ = false;
  bool has_odometry_ = false;

  std::vector<float> observation_;
  std::array<float, 3> last_thrust_body_;

  void loadModel() {
    if (model_path_.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "model_path is empty; RL thrust controller will not publish setpoints");
      return;
    }

    try {
      session_options_.SetIntraOpNumThreads(1);
      session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), session_options_);

      Ort::AllocatorWithDefaultOptions allocator;
      if (input_name_.empty()) {
        auto name = session_->GetInputNameAllocated(0, allocator);
        input_name_ = name.get();
      }
      if (output_name_.empty()) {
        auto name = session_->GetOutputNameAllocated(0, allocator);
        output_name_ = name.get();
      }

      model_ready_ = true;
      RCLCPP_INFO(get_logger(), "Loaded RL policy: %s input='%s' output='%s'", model_path_.c_str(),
                  input_name_.c_str(), output_name_.c_str());
    } catch (const Ort::Exception& error) {
      RCLCPP_ERROR(get_logger(), "Failed to load ONNX policy: %s", error.what());
    }
  }

  void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    has_odometry_ = true;
    std::fill(observation_.begin(), observation_.end(), 0.0f);

    if (observation_size_ >= 3) {
      observation_[0] = msg->position[0];
      observation_[1] = msg->position[1];
      observation_[2] = msg->position[2];
    }
    if (observation_size_ >= 6) {
      observation_[3] = msg->velocity[0];
      observation_[4] = msg->velocity[1];
      observation_[5] = msg->velocity[2];
    }
    if (observation_size_ >= 10) {
      observation_[6] = msg->q[0];
      observation_[7] = msg->q[1];
      observation_[8] = msg->q[2];
      observation_[9] = msg->q[3];
    }
  }

  void timerCallback() {
    if (!model_ready_ || !has_odometry_) { return; }

    try {
      const auto thrust_body = runInference();
      publishThrust(thrust_body);
      last_thrust_body_ = thrust_body;
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "RL inference failed: %s",
                           error.what());
      if (hold_last_on_inference_error_) { publishThrust(last_thrust_body_); }
    }
  }

  std::array<float, 3> runInference() {
    std::vector<int64_t> input_shape = {1, static_cast<int64_t>(observation_size_)};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input_tensor =
      Ort::Value::CreateTensor<float>(memory_info, observation_.data(), observation_.size(),
                                      input_shape.data(), input_shape.size());

    const char* input_names[] = {input_name_.c_str()};
    const char* output_names[] = {output_name_.c_str()};
    auto output_tensors =
      session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    if (output_tensors.empty() || !output_tensors[0].IsTensor()) {
      throw std::runtime_error("policy did not return a tensor");
    }

    auto shape_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    const auto action_count = shape_info.GetElementCount();
    if (action_count < 1) { throw std::runtime_error("policy output is empty"); }

    const float* action = output_tensors[0].GetTensorData<float>();
    if (action_count == 1) {
      if (!finite(action[0])) { throw std::runtime_error("scalar thrust action is not finite"); }
      const auto thrust = clamp(action[0], thrust_min_, thrust_max_);
      std::array<float, 3> thrust_body = {0.0f, 0.0f, 0.0f};
      thrust_body[static_cast<size_t>(thrust_axis_)] = thrust_sign_ * thrust;
      return thrust_body;
    }

    std::array<float, 3> thrust_body = {
      clamp(action[0], -1.0f, 1.0f),
      clamp(action[1], -1.0f, 1.0f),
      clamp(action[2], -1.0f, 1.0f),
    };
    if (!finite(thrust_body[0]) || !finite(thrust_body[1]) || !finite(thrust_body[2])) {
      throw std::runtime_error("vector thrust action contains non-finite values");
    }
    return thrust_body;
  }

  void publishThrust(const std::array<float, 3>& thrust_body) {
    px4_msgs::msg::VehicleThrustSetpoint msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.timestamp_sample = msg.timestamp;
    msg.xyz = thrust_body;
    thrust_pub_->publish(msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RlThrustController>());
  rclcpp::shutdown();
  return 0;
}
