#ifndef FSM_NODE_H
#define FSM_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

namespace px4_ros2_ctrl
{

class FSMNode : public rclcpp::Node
{
public:
    FSMNode();

private:
    // sub
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr position_cmd_sub_;
    
    // pub
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_position_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
    
    // 缓存
    px4_msgs::msg::VehicleControlMode current_control_mode_;
    px4_msgs::msg::TrajectorySetpoint position_cmd_;
    bool has_position_cmd_ = false;
    bool has_received_control_mode_ = false;
    rclcpp::Time last_control_mode_time_;
    
    // controller choose
    // !TODO: change controller 
    std::string active_controller_ = "position";
    
    // timer
    rclcpp::TimerBase::SharedPtr relay_timer_;
    rclcpp::TimerBase::SharedPtr arm_timer_;
    
    // fmu state
    uint64_t offboard_counter_ = 0;
    bool is_armed_ = false;
    
    void controlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
    void positionCmdCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);
    void relayCallback();
    void publishOffboardMode();
    void armCallback();
    void sendArmCommand();
    void sendOffboardModeCommand();
    void connectionCheckCallback();
};

} // namespace px4_ros2_ctrl

#endif
