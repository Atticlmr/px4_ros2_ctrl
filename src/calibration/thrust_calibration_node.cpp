// Copyright 2026 px4_ros2_ctrl contributors

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace px4_ros2_ctrl
{

class ThrustCalibrationNode : public rclcpp::Node
{
public:
    ThrustCalibrationNode() : Node("thrust_calibration_node")
    {
        declare_parameter<double>("time_interval", 1.0);
        declare_parameter<double>("min_battery_voltage", 13.2);
        declare_parameter<double>("mass_kg", 1.0);
        declare_parameter<std::string>("output_file", "/tmp/px4_ros2_ctrl_thrust_calibration.csv");
        declare_parameter<std::string>("battery_topic", "/fmu/out/battery_status");
        declare_parameter<std::string>("attitude_setpoint_topic", "/fmu/in/vehicle_attitude_setpoint");
        declare_parameter<std::string>("thrust_setpoint_topic", "/fmu/in/vehicle_thrust_setpoint");
        declare_parameter<bool>("use_thrust_setpoint_topic", false);
        declare_parameter<int>("thrust_axis", 2);
        declare_parameter<double>("thrust_sign", -1.0);

        time_average_interval_s_ = get_parameter("time_interval").as_double();
        min_battery_voltage_ = get_parameter("min_battery_voltage").as_double();
        mass_kg_ = get_parameter("mass_kg").as_double();
        output_file_ = get_parameter("output_file").as_string();
        use_thrust_setpoint_topic_ = get_parameter("use_thrust_setpoint_topic").as_bool();
        thrust_axis_ = get_parameter("thrust_axis").as_int();
        thrust_sign_ = get_parameter("thrust_sign").as_double();

        thrust_axis_ = std::max(0, std::min(2, thrust_axis_));

        const auto battery_topic = get_parameter("battery_topic").as_string();
        const auto attitude_setpoint_topic = get_parameter("attitude_setpoint_topic").as_string();
        const auto thrust_setpoint_topic = get_parameter("thrust_setpoint_topic").as_string();

        battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
            battery_topic,
            rclcpp::QoS(10).best_effort(),
            std::bind(&ThrustCalibrationNode::batteryCallback, this, std::placeholders::_1));

        attitude_setpoint_sub_ = create_subscription<px4_msgs::msg::VehicleAttitudeSetpoint>(
            attitude_setpoint_topic,
            rclcpp::QoS(10),
            std::bind(&ThrustCalibrationNode::attitudeSetpointCallback, this, std::placeholders::_1));

        thrust_setpoint_sub_ = create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
            thrust_setpoint_topic,
            rclcpp::QoS(10),
            std::bind(&ThrustCalibrationNode::thrustSetpointCallback, this, std::placeholders::_1));

        trigger_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/traj_start_trigger",
            rclcpp::QoS(10),
            std::bind(&ThrustCalibrationNode::triggerCallback, this, std::placeholders::_1));

        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/start",
            std::bind(
                &ThrustCalibrationNode::startCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/stop",
            std::bind(
                &ThrustCalibrationNode::stopCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            std::bind(
                &ThrustCalibrationNode::resetCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        last_record_time_ = now();

        RCLCPP_INFO(get_logger(), "Thrust calibration node started");
        RCLCPP_INFO(get_logger(), "Output file: %s", output_file_.c_str());
        RCLCPP_INFO(
            get_logger(),
            "Call /thrust_calibration_node/start to record and /thrust_calibration_node/stop to save");
    }

private:
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

    std::vector<double> voltage_buffer_;
    std::vector<double> command_buffer_;
    std::vector<double> voltage_records_;
    std::vector<double> command_records_;

    rclcpp::Time last_record_time_;
    bool recording_ = false;
    bool saved_ = false;
    double time_average_interval_s_ = 1.0;
    double min_battery_voltage_ = 13.2;
    double mass_kg_ = 1.0;
    double thrust_sign_ = -1.0;
    int thrust_axis_ = 2;
    bool use_thrust_setpoint_topic_ = false;
    std::string output_file_;

    void batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
    {
        if (!recording_) {
            return;
        }

        if (!msg->connected || msg->voltage_v <= 0.0f) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring invalid battery sample: connected=%d voltage=%.3f",
                msg->connected, msg->voltage_v);
            return;
        }

        voltage_buffer_.push_back(msg->voltage_v);
        flushAverageIfNeeded();

        if (msg->voltage_v < min_battery_voltage_) {
            RCLCPP_WARN(
                get_logger(),
                "Battery voltage %.3f V is below threshold %.3f V; stopping calibration",
                msg->voltage_v,
                min_battery_voltage_);
            stopAndSave();
        }
    }

    void attitudeSetpointCallback(const px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr msg)
    {
        if (!recording_ || use_thrust_setpoint_topic_) {
            return;
        }
        recordCommand(thrust_sign_ * msg->thrust_body[thrust_axis_]);
    }

    void thrustSetpointCallback(const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg)
    {
        if (!recording_ || !use_thrust_setpoint_topic_) {
            return;
        }
        recordCommand(thrust_sign_ * msg->xyz[thrust_axis_]);
    }

    void triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        (void)msg;
        if (!recording_) {
            startRecording();
        } else {
            stopAndSave();
        }
    }

    void startCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (recording_) {
            response->success = false;
            response->message = "already recording";
            return;
        }
        startRecording();
        response->success = true;
        response->message = "started thrust calibration recording";
    }

    void stopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (!recording_) {
            response->success = false;
            response->message = "not recording";
            return;
        }
        const bool ok = stopAndSave();
        response->success = ok;
        response->message = ok ? "saved thrust calibration data" : "failed to save data";
    }

    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        recording_ = false;
        saved_ = false;
        voltage_buffer_.clear();
        command_buffer_.clear();
        voltage_records_.clear();
        command_records_.clear();
        last_record_time_ = now();
        response->success = true;
        response->message = "cleared buffered thrust calibration data";
    }

    void startRecording()
    {
        recording_ = true;
        saved_ = false;
        voltage_buffer_.clear();
        command_buffer_.clear();
        voltage_records_.clear();
        command_records_.clear();
        last_record_time_ = now();
        RCLCPP_INFO(get_logger(), "Started thrust calibration recording");
    }

    bool stopAndSave()
    {
        if (saved_) {
            return true;
        }

        flushAverage(true);
        recording_ = false;
        saved_ = saveData();
        return saved_;
    }

    void recordCommand(double command)
    {
        if (!std::isfinite(command)) {
            return;
        }
        command_buffer_.push_back(command);
    }

    void flushAverageIfNeeded()
    {
        if ((now() - last_record_time_).seconds() < time_average_interval_s_) {
            return;
        }
        flushAverage(false);
    }

    void flushAverage(bool force)
    {
        if (voltage_buffer_.empty() || command_buffer_.empty()) {
            if (force) {
                RCLCPP_WARN(
                    get_logger(),
                    "No complete voltage/thrust sample pair to flush");
            }
            return;
        }

        const double voltage = mean(voltage_buffer_);
        const double command = mean(command_buffer_);
        voltage_records_.push_back(voltage);
        command_records_.push_back(command);
        voltage_buffer_.clear();
        command_buffer_.clear();
        last_record_time_ = now();

        RCLCPP_INFO(get_logger(), "Recorded average: voltage=%.3f V thrust_cmd=%.4f", voltage, command);
    }

    bool saveData()
    {
        if (voltage_records_.empty() || command_records_.empty()) {
            RCLCPP_WARN(get_logger(), "No thrust calibration records to save");
            return false;
        }

        std::ofstream file(output_file_, std::ios::app);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open output file: %s", output_file_.c_str());
            return false;
        }

        file << timestampString()
             << ",mass(kg):," << mass_kg_
             << ",commands,voltage\n";

        const auto count = std::min(command_records_.size(), voltage_records_.size());
        for (std::size_t i = 0; i < count; ++i) {
            file << command_records_[i] << "," << voltage_records_[i] << "\n";
        }

        RCLCPP_INFO(get_logger(), "Stored %zu records to %s", count, output_file_.c_str());
        return true;
    }

    static double mean(const std::vector<double> &values)
    {
        const double sum = std::accumulate(values.begin(), values.end(), 0.0);
        return sum / static_cast<double>(values.size());
    }

    static std::string timestampString()
    {
        const std::time_t now_time = std::time(nullptr);
        char buffer[32];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now_time));
        return std::string(buffer);
    }
};

}  // namespace px4_ros2_ctrl

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2_ctrl::ThrustCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
