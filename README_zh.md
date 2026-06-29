# px4_ros2_ctrl 中文说明

`px4_ros2_ctrl` 是一个用于 PX4 Offboard 控制的 ROS 2 功能包。它的核心目标不是把所有控制、规划、避障逻辑都放在一个节点里，而是把系统分成几层：

```text
控制器 / NMPC / RL / 轨迹跟踪器
  -> /controller/<name>/output
  -> fsm_node
  -> Px4OutputAdapter
  -> /fmu/in/*
  -> PX4
```

控制器只负责产生控制输出，不直接切 PX4 模式、不解锁、不发布 Offboard 心跳。`fsm_node` 负责安全状态机和 Offboard 流程，`Px4OutputAdapter` 负责把统一控制输出转换成 PX4 的 `/fmu/in/*` 消息。

## 当前节点

常用节点如下：

```text
fsm_node
  PX4 Offboard 状态机和安全监督节点

position_controller
  简单位置控制 demo，发布 /controller/position/output

body_rate_nmpc_controller.py
  BODY_RATE + THRUST NMPC demo，发布 /controller/body_rate/output

rl_thrust_controller
  ONNX Runtime 推力策略节点，发布 /controller/thrust/output

thrust_calibration_node
  推力/电压数据记录节点，用于拟合推力参数
```

## 编译


```bash
cd /home/li/Desktop/ws_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_ros2_ctrl
source install/setup.bash
```

如果改了 CMake、launch 或源码后行为不对，可以只清理本包重新编译：

```bash
cd /home/li/Desktop/ws_ros2
rm -rf build/px4_ros2_ctrl install/px4_ros2_ctrl
colcon build --packages-select px4_ros2_ctrl
source install/setup.bash
```

## PX4 SITL 启动流程

推荐启动顺序：

```text
1. 启动 Micro XRCE-DDS Agent
2. 启动 PX4 SITL
3. 确认 /fmu/out/* 话题存在
4. 启动 px4_ros2_ctrl launch
5. 手动解锁或设置 allow_auto_arm
6. 调用 /fsm_node/start_offboard
```

终端 1：

```bash
MicroXRCEAgent udp4 -p 8888
```

终端 2：

```bash
cd /home/li/PX4-Autopilot
make px4_sitl gz_x500
```

终端 3 检查 ROS 2 是否收到 PX4 话题：

```bash
cd /home/li/Desktop/ws_ros2
source install/setup.bash
ros2 topic list | grep /fmu/out
```

至少应能看到：

```text
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/manual_control_setpoint
```

## 运行位置控制 Demo

启动 FSM 和位置控制器：

```bash
cd /home/li/Desktop/ws_ros2
source install/setup.bash
ros2 launch px4_ros2_ctrl fsm_position_control.launch.py
```

这个 launch 会启动：

```text
/fsm_node
/position_controller
```

`position_controller` 会发布正方形目标点：

```text
/controller/position/output
```

默认目标点是 PX4 NED 坐标系下的：

```text
(0, 0, -2)
(5, 0, -2)
(5, 5, -2)
(0, 5, -2)
```

注意：**启动 launch 不等于进入 Offboard，也不等于起飞。** FSM 默认会停在 `STANDBY`，等待你显式调用 service。

## FSM 运行逻辑

`fsm_node` 的状态如下：

```text
WAIT_FOR_PX4
STANDBY
OFFBOARD_PREPARE
OFFBOARD_REQUESTED
OFFBOARD_ACTIVE
MANUAL_OVERRIDE
FAILSAFE
```

正常流程：

```text
启动 fsm_position_control.launch.py
  -> fsm_node 启动
  -> position_controller 开始发布 /controller/position/output
  -> FSM 等待 PX4/local position 健康
  -> WAIT_FOR_PX4 -> STANDBY
  -> 用户调用 /fsm_node/start_offboard
  -> STANDBY -> OFFBOARD_PREPARE
  -> FSM 发布 Offboard heartbeat 和 setpoint，持续 offboard_prepare_s
  -> FSM 发送 PX4 Offboard 模式切换命令
  -> OFFBOARD_PREPARE -> OFFBOARD_REQUESTED
  -> PX4 确认进入 Offboard
  -> OFFBOARD_REQUESTED -> OFFBOARD_ACTIVE
  -> FSM 持续发布 heartbeat 和 setpoint
```

各状态含义：

```text
WAIT_FOR_PX4
  等待 PX4 control mode 和 local position 数据有效

STANDBY
  空闲等待状态。控制器可以已经在发布目标点，但 FSM 不会发 /fmu/in/*

OFFBOARD_PREPARE
  先发布 /fmu/in/offboard_control_mode 和 setpoint，满足 PX4 Offboard 预热要求

OFFBOARD_REQUESTED
  已发送 Offboard 模式命令，等待 PX4 反馈进入 Offboard

OFFBOARD_ACTIVE
  正常 Offboard 控制，持续转发控制器输出

MANUAL_OVERRIDE
  手动接管锁定状态，不会自动重新进入 Offboard

FAILSAFE
  超时或失效状态，停止正常 Offboard 输出
```

## 必须调用的服务

启动 Offboard 请求：

```bash
ros2 service call /fsm_node/start_offboard std_srvs/srv/Trigger
```

正常会看到类似状态变化：

```text
STANDBY -> OFFBOARD_PREPARE
OFFBOARD_PREPARE -> OFFBOARD_REQUESTED
OFFBOARD_REQUESTED -> OFFBOARD_ACTIVE
```

停止 Offboard，并锁定手动接管：

```bash
ros2 service call /fsm_node/stop_offboard std_srvs/srv/Trigger
```

重置手动接管或 failsafe 锁定：

```bash
ros2 service call /fsm_node/reset_override std_srvs/srv/Trigger
```

重置后如果想重新进入 Offboard，需要再次调用：

```bash
ros2 service call /fsm_node/start_offboard std_srvs/srv/Trigger
```

## 解锁和自动解锁

默认参数：

```yaml
allow_auto_arm: false
```

所以默认情况下，FSM 不会自动解锁。你需要在 QGroundControl、PX4 shell 或其他方式里手动 arm。

如果只是 SITL 调试，可以把 launch 里的参数改成：

```yaml
allow_auto_arm: true
```

这样 FSM 在发送 Offboard 模式命令时，如果 PX4 还没有 arm，会同时发送 arm command。

注意：`allow_auto_arm` 只控制自动解锁，不代表自动进入 Offboard。进入 Offboard 仍然需要调用：

```bash
ros2 service call /fsm_node/start_offboard std_srvs/srv/Trigger
```

## FSM 参数

默认 position launch 参数：

```yaml
active_controller: position
allow_auto_arm: false
land_on_failsafe: false
controller_timeout_s: 0.25
px4_timeout_s: 3.0
estimator_timeout_s: 0.5
offboard_prepare_s: 1.1
```

参数说明：

```text
active_controller
  当前启用的控制器。position launch 使用 position。

allow_auto_arm
  是否在请求 Offboard 时自动发送解锁命令。

land_on_failsafe
  进入 FAILSAFE 时是否发送一次 PX4 land command。

controller_timeout_s
  控制器输出最大允许年龄。超过后认为控制器输出失效。

px4_timeout_s
  PX4 control mode 数据最大允许年龄。

estimator_timeout_s
  local position 数据最大允许年龄。

offboard_prepare_s
  发送 Offboard 模式命令前，先持续发布 heartbeat/setpoint 的时间。
  PX4 要求进入 Offboard 前先收到超过 1 秒的 Offboard proof-of-life。
```

## Position 控制路径

Position demo 的路径是：

```text
position_controller
  -> /controller/position/output
  -> FSMNode::positionOutputCallback()
  -> ControllerOutput(level=POSITION)
  -> Px4OutputAdapter
  -> /fmu/in/offboard_control_mode, position=true
  -> /fmu/in/trajectory_setpoint
  -> PX4 内置位置控制器
```

也就是说，`position_controller` 只是发位置目标点，真正的位置环、速度环、姿态环仍然由 PX4 内部控制器完成。

## BODY_RATE + THRUST NMPC Demo

运行：

```bash
cd /home/li/Desktop/ws_ros2
source install/setup.bash
ros2 launch px4_ros2_ctrl fsm_body_rate_nmpc.launch.py
```

NMPC 控制路径：

```text
/fmu/out/vehicle_odometry
  -> body_rate_nmpc_controller.py
  -> acados 求解 NMPC
  -> /controller/body_rate/output
  -> fsm_node
  -> /fmu/in/offboard_control_mode, body_rate=true
  -> /fmu/in/vehicle_rates_setpoint
  -> PX4 角速度控制器 / 混控 / 电机
```

NMPC 输出：

```text
roll_rate
pitch_rate
yaw_rate
normalized thrust
```

默认参考目标是一个 3m x 3m 的正方形：

```text
(0, 0, -2)
(3, 0, -2)
(3, 3, -2)
(0, 3, -2)
```

这个 demo 的作用是验证 BODY_RATE + THRUST Offboard 控制链路。它比 position 控制更依赖模型参数、推力标定和坐标系一致性。

生成 acados solver：

```bash
cd /home/li/Desktop/ws_ros2/src/px4_ros2_ctrl
PYTHONPATH=$PWD/third_party/acados/interfaces/acados_template:$PWD/third_party/casadi/build/lib \
LD_LIBRARY_PATH=$PWD/third_party/acados/lib:$PWD/third_party/casadi/build/lib \
ACADOS_SOURCE_DIR=$PWD/third_party/acados \
./scripts/generate_body_rate_nmpc_solver.py
```

测试 solver：

```bash
cd /home/li/Desktop/ws_ros2/src/px4_ros2_ctrl
PYTHONPATH=$PWD/third_party/acados/interfaces/acados_template:$PWD/third_party/casadi/build/lib \
LD_LIBRARY_PATH=$PWD/third_party/acados/lib:$PWD/third_party/casadi/build/lib \
ACADOS_SOURCE_DIR=$PWD/third_party/acados \
./scripts/test_body_rate_nmpc_solver.py
```

## RL 推力控制器

运行：

```bash
cd /home/li/Desktop/ws_ros2
source install/setup.bash
ros2 launch px4_ros2_ctrl fsm_rl_thrust.launch.py model_path:=/absolute/path/to/policy.onnx
```

路径：

```text
/fmu/out/vehicle_odometry
  -> rl_thrust_controller
  -> /controller/thrust/output
  -> fsm_node
  -> /fmu/in/offboard_control_mode, thrust_and_torque=true
  -> /fmu/in/vehicle_thrust_setpoint
  -> /fmu/in/vehicle_torque_setpoint
```

注意：纯推力控制器不控制姿态、yaw 或 body rate，不是完整的多旋翼控制器。它主要用于接口测试、垂向推力实验或特定策略验证。

## 推力标定

记录推力/电压数据：

```bash
cd /home/li/Desktop/ws_ros2
source install/setup.bash
ros2 run px4_ros2_ctrl thrust_calibration_node
```

开始记录：

```bash
ros2 service call /thrust_calibration_node/start std_srvs/srv/Trigger
```

停止并保存：

```bash
ros2 service call /thrust_calibration_node/stop std_srvs/srv/Trigger
```

清空缓存：

```bash
ros2 service call /thrust_calibration_node/reset std_srvs/srv/Trigger
```

拟合参数：

```bash
ros2 run px4_ros2_ctrl fit_thrust_calibration.py /tmp/thrust_calibration.csv
```

拟合得到的核心参数包括：

```text
hover_command
nominal_voltage
voltage_compensation_exponent
newtons_per_normalized_command_at_nominal_voltage
```

这些参数用于把归一化推力命令、实际电池电压和近似升力关系对应起来。

## 添加新控制器

新控制器不要直接发布 `/fmu/in/*`。推荐做法：

```text
1. 新建控制器节点
2. 订阅需要的状态估计或轨迹输入
3. 发布统一控制输出到 /controller/<name>/output
4. 在 FSM 里增加对应 callback
5. 在 Px4OutputAdapter 里增加对应 PX4 setpoint 映射
```

当前已有控制输出类型：

```text
POSITION
  /controller/position/output
  px4_msgs/msg/TrajectorySetpoint

BODY_RATE
  /controller/body_rate/output
  px4_msgs/msg/VehicleRatesSetpoint

THRUST
  /controller/thrust/output
  px4_msgs/msg/VehicleThrustSetpoint
```

## 实机注意事项

实机前必须先在 SITL 验证：

```text
RC 遥控链路可用
有实体模式开关可以切回 Position / Altitude / Manual
PX4 Offboard loss 参数已配置
控制器输出有幅值限制
坐标系确认是 PX4 NED / body-FRD
allow_auto_arm 默认保持 false
手动接管后 FSM 会进入 MANUAL_OVERRIDE，不会自动重新进入 Offboard
```

实机不要一开始就启用 BODY_RATE + THRUST NMPC。建议顺序是：

```text
1. position_controller SITL
2. position_controller 实机低风险测试
3. BODY_RATE NMPC SITL
4. 推力标定
5. BODY_RATE NMPC 绑桨/台架检查
6. 小范围实机测试
```
