  #include <rclcpp/rclcpp.hpp>
  #include <px4_msgs/msg/trajectory_setpoint.hpp>
  #include <px4_msgs/msg/vehicle_local_position.hpp>

  using namespace std::chrono_literals;

  class PositionControllerSimple : public rclcpp::Node
  {
  public:
      PositionControllerSimple() : Node("position_controller_simple")
      {
          // 发布：给FSM的目标点（不是直接给PX4！）
          cmd_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
              "/controller/position_cmd", 10);

          // 订阅：当前位置（用于判断到达）
          pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
              "/fmu/out/vehicle_local_position",
              rclcpp::QoS(10).best_effort(),
              [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                  current_x_ = msg->x;
                  current_y_ = msg->y;
                  current_z_ = msg->z;
                  has_position_ = true;
              });

          // 定时器：10Hz发布目标点
          timer_ = this->create_wall_timer(100ms, [this]() { this->publishTarget(); });

          // 目标点：正方形轨迹的四个点
          targets_ = {
              {0.0, 0.0, -2.0},   // 点1：原点上方2米
              {5.0, 0.0, -2.0},   // 点2：向前5米
              {5.0, 5.0, -2.0},   // 点3：向右5米
              {0.0, 5.0, -2.0},   // 点4：返回
          };
          target_index_ = 0;

          RCLCPP_INFO(get_logger(), "Position controller started");
          RCLCPP_INFO(get_logger(), "Publishing to /controller/position_cmd");
      }

  private:
      rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr cmd_pub_;
      rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
      rclcpp::TimerBase::SharedPtr timer_;

      std::vector<std::array<float, 3>> targets_;
      size_t target_index_;

      float current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;
      bool has_position_ = false;

      void publishTarget()
      {
          if (!has_position_) return;

          auto &target = targets_[target_index_];

          // 检查是否到达当前目标（距离<0.5米）
          float dx = target[0] - current_x_;
          float dy = target[1] - current_y_;
          float dz = target[2] - current_z_;
          float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

          if (dist < 0.5f) {
              // 切换到下一个点
              target_index_ = (target_index_ + 1) % targets_.size();
              RCLCPP_INFO(get_logger(), "Reached target, switching to point %zu", target_index_);
          }

          // 发布目标点
          px4_msgs::msg::TrajectorySetpoint msg;
          msg.timestamp = this->now().nanoseconds() / 1000;
          msg.position[0] = target[0];
          msg.position[1] = target[1];
          msg.position[2] = target[2];
          msg.yaw = 0.0;  // 保持朝北

          // 速度设为NaN，表示纯位置控制
          msg.velocity[0] = std::nanf("");
          msg.velocity[1] = std::nanf("");
          msg.velocity[2] = std::nanf("");

          cmd_pub_->publish(msg);
      }
  };

  int main(int argc, char *argv[])
  {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<PositionControllerSimple>());
      rclcpp::shutdown();
      return 0;
  }