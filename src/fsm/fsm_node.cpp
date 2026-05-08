// Copyright 2026 px4_ros2_ctrl contributors

#include "px4_ros2_ctrl/fsm_node.h"

#include <utility>

namespace px4_ros2_ctrl
{

FSMNode::FSMNode() : Node("fsm_node")
{
    declare_parameter<std::string>("active_controller", "position");
    declare_parameter<double>("px4_timeout_s", 3.0);
    declare_parameter<double>("estimator_timeout_s", 0.5);
    declare_parameter<double>("controller_timeout_s", 0.25);
    declare_parameter<double>("offboard_prepare_s", 1.1);
    declare_parameter<bool>("allow_auto_arm", false);
    declare_parameter<bool>("land_on_failsafe", false);

    active_controller_ = get_parameter("active_controller").as_string();
    px4_timeout_s_ = get_parameter("px4_timeout_s").as_double();
    estimator_timeout_s_ = get_parameter("estimator_timeout_s").as_double();
    controller_timeout_s_ = get_parameter("controller_timeout_s").as_double();
    offboard_prepare_s_ = get_parameter("offboard_prepare_s").as_double();
    allow_auto_arm_ = get_parameter("allow_auto_arm").as_bool();
    land_on_failsafe_ = get_parameter("land_on_failsafe").as_bool();

    px4_adapter_ = std::make_unique<Px4OutputAdapter>(this);

    control_mode_sub_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode",
        rclcpp::QoS(10).best_effort(),
        std::bind(&FSMNode::controlModeCallback, this, std::placeholders::_1));

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        rclcpp::QoS(10).best_effort(),
        std::bind(&FSMNode::localPositionCallback, this, std::placeholders::_1));

    manual_control_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        "/fmu/out/manual_control_setpoint",
        rclcpp::QoS(10).best_effort(),
        std::bind(&FSMNode::manualControlCallback, this, std::placeholders::_1));

    position_output_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/controller/position/output",
        rclcpp::QoS(10),
        std::bind(&FSMNode::positionOutputCallback, this, std::placeholders::_1));

    start_offboard_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/start_offboard",
        std::bind(
            &FSMNode::startOffboardCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    stop_offboard_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/stop_offboard",
        std::bind(
            &FSMNode::stopOffboardCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    reset_override_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_override",
        std::bind(
            &FSMNode::resetOverrideCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    state_enter_time_ = now();
    last_control_mode_time_ = now();
    last_local_position_time_ = now();
    last_controller_output_time_ = now();

    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&FSMNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "FSM supervisor started with controller '%s'", active_controller_.c_str());
    RCLCPP_INFO(
        get_logger(),
        "Call /fsm_node/start_offboard to request Offboard control");
}

void FSMNode::controlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
    const bool was_offboard = has_control_mode_ && current_control_mode_.flag_control_offboard_enabled;
    current_control_mode_ = *msg;
    has_control_mode_ = true;
    last_control_mode_time_ = now();

    if (offboardState() && was_offboard && !msg->flag_control_offboard_enabled) {
        manual_override_latched_ = true;
        transitionTo(State::MANUAL_OVERRIDE, "PX4 left Offboard mode");
    }

    if (offboardState() && msg->flag_control_manual_enabled) {
        manual_override_latched_ = true;
        transitionTo(State::MANUAL_OVERRIDE, "manual control mode selected");
    }
}

void FSMNode::localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    current_local_position_ = *msg;
    has_local_position_ = true;
    last_local_position_time_ = now();
}

void FSMNode::manualControlCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
{
    if (offboardState() && msg->valid && msg->sticks_moving) {
        manual_override_latched_ = true;
        transitionTo(State::MANUAL_OVERRIDE, "manual sticks moved");
    }
}

void FSMNode::positionOutputCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
    ControllerOutput output;
    output.level = ControlLevel::POSITION;
    output.stamp = now();
    output.valid = true;
    output.controller_name = "position";
    output.position = msg->position;
    output.velocity = msg->velocity;
    output.acceleration = msg->acceleration;
    output.yaw = msg->yaw;
    output.yawspeed = msg->yawspeed;

    controller_output_ = output;
    active_control_level_ = output.level;
    has_controller_output_ = true;
    last_controller_output_time_ = now();
}

void FSMNode::startOffboardCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    if (manual_override_latched_) {
        response->success = false;
        response->message = "manual override is latched; call reset_override first";
        return;
    }

    has_start_request_ = true;
    response->success = true;
    response->message = "Offboard start requested";
}

void FSMNode::stopOffboardCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    has_start_request_ = false;
    manual_override_latched_ = true;
    transitionTo(State::MANUAL_OVERRIDE, "stop_offboard service called");
    response->success = true;
    response->message = "Offboard stopped and manual override latched";
}

void FSMNode::resetOverrideCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    manual_override_latched_ = false;
    has_start_request_ = false;
    if (state_ == State::MANUAL_OVERRIDE || state_ == State::FAILSAFE) {
        transitionTo(State::STANDBY, "manual override reset");
    }
    response->success = true;
    response->message = "manual override reset; call start_offboard to resume";
}

void FSMNode::controlLoop()
{
    if (state_ != previous_state_) {
        RCLCPP_INFO(get_logger(), "FSM state: %s", stateName(state_));
        previous_state_ = state_;
    }

    if (!px4ConnectionHealthy()) {
        if (offboardState()) {
            transitionTo(State::FAILSAFE, "PX4 connection timeout during Offboard");
        } else if (state_ != State::WAIT_FOR_PX4 && state_ != State::FAILSAFE) {
            transitionTo(State::WAIT_FOR_PX4, "PX4 connection timeout");
        }
    }

    if (state_ == State::OFFBOARD_ACTIVE && (!estimatorHealthy() || !controllerOutputFresh())) {
        transitionTo(State::FAILSAFE, "estimator or controller output timeout");
    }

    if (manual_override_latched_ && offboardState()) {
        transitionTo(State::MANUAL_OVERRIDE, "manual override latched");
    }

    switch (state_) {
    case State::WAIT_FOR_PX4:
        if (px4ConnectionHealthy() && estimatorHealthy()) {
            transitionTo(State::STANDBY, "PX4 and estimator are healthy");
        }
        break;

    case State::STANDBY:
        if (has_start_request_ && !manual_override_latched_) {
            if (!controllerOutputFresh()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Waiting for fresh controller output before Offboard prepare");
                break;
            }
            transitionTo(State::OFFBOARD_PREPARE, "start request accepted");
        }
        break;

    case State::OFFBOARD_PREPARE:
        px4_adapter_->publishOffboardHeartbeat(active_control_level_);
        if (!controllerOutputFresh()) {
            transitionTo(State::FAILSAFE, "controller output timed out during Offboard prepare");
            break;
        }
        px4_adapter_->publishSetpoint(controller_output_);
        if ((now() - state_enter_time_).seconds() >= offboard_prepare_s_) {
            px4_adapter_->sendOffboardModeCommand();
            if (allow_auto_arm_ && !px4Armed()) {
                px4_adapter_->sendArmCommand(true);
            }
            transitionTo(State::OFFBOARD_REQUESTED, "Offboard mode command sent");
        }
        break;

    case State::OFFBOARD_REQUESTED:
        px4_adapter_->publishOffboardHeartbeat(active_control_level_);
        if (controllerOutputFresh()) {
            px4_adapter_->publishSetpoint(controller_output_);
        } else {
            transitionTo(State::FAILSAFE, "controller output timed out while waiting for Offboard");
            break;
        }
        if (px4InOffboard()) {
            transitionTo(State::OFFBOARD_ACTIVE, "PX4 entered Offboard");
        }
        break;

    case State::OFFBOARD_ACTIVE:
        px4_adapter_->publishOffboardHeartbeat(active_control_level_);
        px4_adapter_->publishSetpoint(controller_output_);
        break;

    case State::MANUAL_OVERRIDE:
        has_start_request_ = false;
        break;

    case State::FAILSAFE:
        has_start_request_ = false;
        if (land_on_failsafe_) {
            px4_adapter_->sendLandCommand();
            land_on_failsafe_ = false;
        }
        break;
    }
}

void FSMNode::transitionTo(State next_state, const std::string &reason)
{
    if (state_ == next_state) {
        return;
    }

    RCLCPP_WARN(
        get_logger(), "FSM transition %s -> %s: %s",
        stateName(state_), stateName(next_state), reason.c_str());
    state_ = next_state;
    state_enter_time_ = now();
}

bool FSMNode::px4ConnectionHealthy() const
{
    if (!has_control_mode_) {
        return false;
    }
    const auto current_time = now();
    return (current_time - last_control_mode_time_).seconds() <= px4_timeout_s_;
}

bool FSMNode::estimatorHealthy() const
{
    if (!has_local_position_) {
        return false;
    }
    return (now() - last_local_position_time_).seconds() <= estimator_timeout_s_ &&
           current_local_position_.xy_valid &&
           current_local_position_.z_valid;
}

bool FSMNode::controllerOutputFresh() const
{
    return has_controller_output_ &&
           controller_output_.valid &&
           (now() - last_controller_output_time_).seconds() <= controller_timeout_s_;
}

bool FSMNode::px4InOffboard() const
{
    return has_control_mode_ && current_control_mode_.flag_control_offboard_enabled;
}

bool FSMNode::px4InManualMode() const
{
    return has_control_mode_ && current_control_mode_.flag_control_manual_enabled;
}

bool FSMNode::px4Armed() const
{
    return has_control_mode_ && current_control_mode_.flag_armed;
}

bool FSMNode::offboardState() const
{
    return state_ == State::OFFBOARD_PREPARE ||
           state_ == State::OFFBOARD_REQUESTED ||
           state_ == State::OFFBOARD_ACTIVE;
}

const char *FSMNode::stateName(State state) const
{
    switch (state) {
    case State::WAIT_FOR_PX4:
        return "WAIT_FOR_PX4";
    case State::STANDBY:
        return "STANDBY";
    case State::OFFBOARD_PREPARE:
        return "OFFBOARD_PREPARE";
    case State::OFFBOARD_REQUESTED:
        return "OFFBOARD_REQUESTED";
    case State::OFFBOARD_ACTIVE:
        return "OFFBOARD_ACTIVE";
    case State::MANUAL_OVERRIDE:
        return "MANUAL_OVERRIDE";
    case State::FAILSAFE:
        return "FAILSAFE";
    }
    return "UNKNOWN";
}

}  // namespace px4_ros2_ctrl

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2_ctrl::FSMNode>());
    rclcpp::shutdown();
    return 0;
}
