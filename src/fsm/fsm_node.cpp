#include "px4_ros2_ctrl/fsm_node.h"

namespace px4_ros2_ctrl
{

FSMNode::FSMNode() : Node("fsm_node")
{
    // 声明参数
    this->declare_parameter<std::string>("active_controller", "position");
    active_controller_ = this->get_parameter("active_controller").as_string();
    
    RCLCPP_INFO(get_logger(), "[FSM init] FSM started!");
    RCLCPP_INFO(get_logger(), "Active controller: %s", active_controller_.c_str());

    // control mode sub 
    // for PX4 armed/offboard/position etc.
    control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode",
        rclcpp::QoS(10).best_effort(),
        std::bind(&FSMNode::controlModeCallback, this, std::placeholders::_1)
    );

    // pub position_controller
    // 10 is sequence length
    // !TODO: more controller
    position_cmd_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/controller/position_cmd", 10,
        std::bind(&FSMNode::positionCmdCallback, this, std::placeholders::_1)
    );

    // 发布到PX4
    px4_position_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    offboard_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // timer default to be 100Hz, 1000/10 =100
    relay_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&FSMNode::relayCallback, this)
    );

    // arm timer 1Hz enough
    arm_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&FSMNode::armCallback, this)
    );

    last_control_mode_time_ = this->now();
    RCLCPP_INFO(get_logger(), "FSM relay initialized (position only)");
}

/*
FSM control mode callback:
judge: 
    1. if connected to fmu: has_received_control_mode_
    2. if fmu is offboard

print:
    heartbreak for FSM
*/
void FSMNode::controlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
    current_control_mode_ = *msg;
    has_received_control_mode_ = true;
    last_control_mode_time_ = this->now();

    std::string arm_state = msg->flag_armed ? "Armed" : "Disarmed";
    
    // 使用控制标志判断模式（更准确）
    std::string fmu_mode;
    if (msg->flag_control_offboard_enabled) {
        fmu_mode = "Offboard";
    } else if (msg->flag_control_manual_enabled) {
        fmu_mode = "Manual";
    } else if (msg->flag_control_auto_enabled) {
        fmu_mode = "Auto";
    } else {
        fmu_mode = "Other";
    }

    std::string position_mode;
    if (msg->flag_control_position_enabled) {
        position_mode = "Position";
    } else if (msg->flag_control_altitude_enabled) {
        position_mode = "Altitude";
    } else if (msg->flag_control_attitude_enabled) {
        position_mode = "Attitude";
    } else {
        position_mode = "Rate";
    }

    RCLCPP_INFO(get_logger(), "[%s] Mode: %s | Control: %s",
                arm_state.c_str(), fmu_mode.c_str(), position_mode.c_str());
}

void FSMNode::positionCmdCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
    position_cmd_ = *msg;
    has_position_cmd_ = true;
}

void FSMNode::relayCallback()
{
    publishOffboardMode();
    
    if (active_controller_ == "position") {
        if (has_position_cmd_) {
            px4_position_pub_->publish(position_cmd_);
        }
    }
}

void FSMNode::publishOffboardMode()
{
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = this->now().nanoseconds() / 1000;
    msg.position = (active_controller_ == "position");
    msg.attitude = false;
    msg.body_rate = false;
    offboard_pub_->publish(msg);
}

void FSMNode::armCallback()
{
    if (offboard_counter_ < 10) {
        offboard_counter_++;
        RCLCPP_INFO(get_logger(), "Sending setpoint %lu/10...", offboard_counter_);
        return;
    }
    
    if (offboard_counter_ == 10) {
        sendOffboardModeCommand();
        offboard_counter_++;
    }
    
    if (current_control_mode_.flag_control_offboard_enabled && !is_armed_) {
        sendArmCommand();
        is_armed_ = true;
        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "DRONE ARMED! Ready to fly!");
        RCLCPP_INFO(get_logger(), "========================================");
    }
}

void FSMNode::sendArmCommand()
{
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = this->now().nanoseconds() / 1000;
    msg.param1 = 1.0;
    msg.param2 = 0.0;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    
    vehicle_cmd_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Send ARM command");
}

void FSMNode::sendOffboardModeCommand()
{
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = this->now().nanoseconds() / 1000;
    msg.param1 = 1.0;
    msg.param2 = 6.0;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    
    vehicle_cmd_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Send OFFBOARD mode command");
}

void FSMNode::connectionCheckCallback()
{
    if (!has_received_control_mode_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Waiting for PX4 connection...");
    } else {
        auto elapsed = (this->now() - last_control_mode_time_).seconds();
        if (elapsed > 3.0) {
            RCLCPP_WARN(get_logger(), "PX4 connection lost! No message for %.1f seconds", elapsed);
        }
    }
}

} // namespace px4_ros2_ctrl

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2_ctrl::FSMNode>());
    rclcpp::shutdown();
    return 0;
}
