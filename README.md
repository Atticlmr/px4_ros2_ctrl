# px4_ros2_ctrl

`px4_ros2_ctrl` is a ROS 2 package for running a PX4 upper-level controller through
Offboard mode. The current architecture separates the safety supervisor, controller
output, and PX4 topic adapter so new controllers such as MPC, SO3, or PX4-native
position control can be added without letting each controller manage arming, mode
switching, or failsafe behavior.

## Architecture

The control path is:

```text
controller node
  -> /controller/<name>/output
  -> fsm_node
  -> Px4OutputAdapter
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
  -> /fmu/in/vehicle_rates_setpoint
  -> /fmu/in/vehicle_thrust_setpoint
  -> /fmu/in/vehicle_torque_setpoint
  -> /fmu/in/vehicle_command
```

Main components:

- `fsm_node`: safety supervisor and finite-state machine. It owns Offboard entry,
  Offboard heartbeat gating, manual override latch, controller timeout handling,
  PX4 connection timeout handling, and estimator validity checks.
- `Px4OutputAdapter`: the only layer that publishes PX4 input topics and vehicle
  commands.
- `position_controller`: simple demo controller. It publishes POSITION output to
  `/controller/position/output`; it does not publish PX4 Offboard heartbeat or
  vehicle commands.
- `body_rate_nmpc_controller.py`: acados/CasADi BODY_RATE + THRUST NMPC demo for
  PX4 SITL. It publishes `/controller/body_rate/output`; the FSM and adapter decide
  whether that output reaches PX4.
- `rl_thrust_controller`: ONNX Runtime C++ inference node for a pure thrust RL
  policy. It publishes `/controller/thrust/output`; it does not publish PX4 mode,
  heartbeat, arming, or vehicle command topics.
- `frame_transforms`: helper library for ROS/PX4 frame conversions.

The package also keeps several PX4 example listener/offboard nodes under `src/lib`.
They are useful references, but the FSM launch path uses `fsm_node` and
`position_controller`.

## Safety Model

PX4 requires an Offboard proof-of-life signal before and during Offboard control.
For ROS 2, this heartbeat is `/fmu/in/offboard_control_mode`. The FSM publishes it
only when it is preparing for, requesting, or actively running Offboard control.

The FSM intentionally stops publishing Offboard heartbeat and setpoints when it no
longer trusts the control loop. This lets PX4 exit Offboard according to PX4 failsafe
parameters such as `COM_OF_LOSS_T` and `COM_OBL_RC_ACT`.

Important behavior:

- Offboard is not started automatically. Call `start_offboard` explicitly.
- Auto-arm is disabled by default. Arm manually in QGroundControl/PX4 shell, or
  set `allow_auto_arm:=true` only after bench and SITL validation.
- Manual override is latched. If PX4 leaves Offboard or manual sticks move during
  Offboard, the FSM enters `MANUAL_OVERRIDE` and will not automatically re-enter
  Offboard.
- After manual override, call `reset_override`, then call `start_offboard` again if
  you want to resume ROS 2 control.
- If controller output, PX4 state, or local position estimator data times out, the
  FSM enters `FAILSAFE` and stops normal Offboard control.

Recommended PX4-side setup:

- Keep a physical RC/mode switch that can force `Position`, `Altitude`, or `Manual`
  mode.
- Configure `COM_OF_LOSS_T` and `COM_OBL_RC_ACT` for the desired Offboard-loss
  behavior.
- Do not disable RC input if you expect a pilot to take over manually.

## Dependencies

- ROS 2 Humble or compatible ROS 2 distribution
- PX4 `px4_msgs`
- PX4 uXRCE-DDS bridge / Micro XRCE-DDS Agent
- `rclcpp`, `std_srvs`, `geometry_msgs`, `sensor_msgs`, `Eigen3`
- Optional for RL controller inference: repo-local ONNX Runtime C++ under
  `third_party/onnxruntime`

PX4 setup references:

- https://docs.px4.io/main/en/middleware/uxrce_dds.html
- https://docs.px4.io/main/en/ros/ros2_comm.html
- https://docs.px4.io/main/en/flight_modes/offboard.html

## Build

Run build from the workspace root, not from this package directory:

```bash
cd /home/li/Desktop/ws_ros2
colcon build --packages-select px4_ros2_ctrl
source install/setup.bash
```

The current code compiles with:

```bash
colcon build --packages-select px4_ros2_ctrl
```

`colcon test --packages-select px4_ros2_ctrl` may fail because the inherited
repository has package-wide lint issues in older example files. Those lint failures
are separate from the FSM build result.

## Run With The Demo Position Controller

Start PX4 SITL or a real PX4 target first, and make sure the ROS 2 bridge is running.
Then launch the FSM and demo position controller:

```bash
cd /home/li/Desktop/ws_ros2
source install/setup.bash
ros2 launch px4_ros2_ctrl fsm_position_control.launch.py
```

The launch file starts:

- `fsm_node`
- `position_controller`

The demo controller publishes a square trajectory as POSITION output on:

```text
/controller/position/output
```

The FSM publishes PX4 input topics only when Offboard is requested and safety checks
pass.

## FSM Runtime Logic

`fsm_node` starts in `WAIT_FOR_PX4`. It subscribes to PX4 status topics,
controller output topics, and manual-control input, but it does not publish PX4
Offboard heartbeat or setpoints immediately after launch.

The normal position-control startup flow is:

```text
launch fsm_position_control.launch.py
  -> fsm_node starts
  -> position_controller starts publishing /controller/position/output
  -> FSM waits for PX4/local-position health
  -> WAIT_FOR_PX4 -> STANDBY
  -> user calls /fsm_node/start_offboard
  -> STANDBY -> OFFBOARD_PREPARE
  -> publish Offboard heartbeat and trajectory setpoint for offboard_prepare_s
  -> send PX4 Offboard mode command
  -> OFFBOARD_PREPARE -> OFFBOARD_REQUESTED
  -> wait until PX4 reports Offboard enabled
  -> OFFBOARD_REQUESTED -> OFFBOARD_ACTIVE
  -> keep publishing heartbeat and setpoint at the FSM timer rate
```

The FSM only forwards the selected controller. For the default position launch,
`active_controller` is `position`, so the FSM accepts `/controller/position/output`
and ignores body-rate or thrust controller topics.

Important state behavior:

- `WAIT_FOR_PX4`: waits for fresh PX4 control-mode data and valid local position.
- `STANDBY`: safe idle state. Controller output may be arriving, but nothing is
  sent to PX4 until `start_offboard` is called.
- `OFFBOARD_PREPARE`: publishes `/fmu/in/offboard_control_mode` and the current
  setpoint before requesting PX4 Offboard mode.
- `OFFBOARD_REQUESTED`: keeps publishing heartbeat/setpoint while waiting for PX4
  to confirm Offboard mode.
- `OFFBOARD_ACTIVE`: normal control state. The FSM continuously forwards fresh
  controller output through `Px4OutputAdapter`.
- `MANUAL_OVERRIDE`: latched manual takeover. The FSM stops normal Offboard output
  and will not restart automatically.
- `FAILSAFE`: controller/PX4/estimator timeout path. Normal Offboard output stops;
  if `land_on_failsafe` is true, one PX4 land command is sent.

Auto-arm is separate from Offboard start. With the default `allow_auto_arm: false`,
you must arm manually before or during the Offboard procedure. If
`allow_auto_arm: true`, the FSM sends an arm command when it sends the Offboard
mode command.

## Start, Stop, And Reset Offboard

Start Offboard request:

```bash
ros2 service call /fsm_node/start_offboard std_srvs/srv/Trigger
```

Stop Offboard and latch manual override:

```bash
ros2 service call /fsm_node/stop_offboard std_srvs/srv/Trigger
```

Reset the manual override/failsafe latch:

```bash
ros2 service call /fsm_node/reset_override std_srvs/srv/Trigger
```

After reset, call `start_offboard` again to resume ROS 2 control.

## FSM Parameters

The default launch file sets conservative values:

```yaml
active_controller: position
allow_auto_arm: false
land_on_failsafe: false
controller_timeout_s: 0.25
px4_timeout_s: 3.0
estimator_timeout_s: 0.5
offboard_prepare_s: 1.1
```

Parameter meaning:

- `active_controller`: selected controller name. Currently `position` is wired.
- `allow_auto_arm`: if true, the FSM sends an arm command when requesting Offboard.
- `land_on_failsafe`: if true, the FSM sends one PX4 land command on failsafe entry.
- `controller_timeout_s`: maximum age of controller output.
- `px4_timeout_s`: maximum age of PX4 control-mode state. This is intentionally
  looser than the controller timeout because some PX4/DDS outputs are not steady
  high-rate topics in all SITL configurations.
- `estimator_timeout_s`: maximum age of local position estimator data.
- `offboard_prepare_s`: heartbeat warm-up time before sending the Offboard mode
  command. PX4 requires more than 1 second of valid Offboard proof-of-life before
  entering Offboard.

## BODY_RATE + THRUST NMPC Demo

The NMPC demo is built around the PX4 Gazebo x500 SITL model:

- airframe: `/home/li/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`
- model: `/home/li/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf`
- base inertial model: `/home/li/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`

The default model uses mass `2.0 kg`, inertia approximately
`Ixx=0.0216667`, `Iyy=0.0216667`, `Izz=0.04`, and PX4 hover thrust
`MPC_THR_HOVER=0.60`.

Generate the acados C solver from CasADi model code:

```bash
cd /home/li/Desktop/ws_ros2/src/px4_ros2_ctrl
PYTHONPATH=$PWD/third_party/acados/interfaces/acados_template:$PWD/third_party/casadi/build/lib \
LD_LIBRARY_PATH=$PWD/third_party/acados/lib:$PWD/third_party/casadi/build/lib \
ACADOS_SOURCE_DIR=$PWD/third_party/acados \
./scripts/generate_body_rate_nmpc_solver.py
```

Smoke-test the generated solver:

```bash
./scripts/test_body_rate_nmpc_solver.py
```

Build from the workspace root:

```bash
cd /home/li/Desktop/ws_ros2
colcon build --packages-select px4_ros2_ctrl
source install/setup.bash
```

Start PX4 SITL and the DDS bridge first. Then run the NMPC FSM demo:

```bash
ros2 launch px4_ros2_ctrl fsm_body_rate_nmpc.launch.py
```

Request Offboard after PX4 and the estimator are healthy:

```bash
ros2 service call /fsm_node/start_offboard std_srvs/srv/Trigger
```

The NMPC node subscribes to:

```text
/fmu/out/vehicle_odometry
```

The NMPC node publishes:

```text
/controller/body_rate/output
```

The FSM adapter publishes to PX4:

```text
/fmu/in/offboard_control_mode
/fmu/in/vehicle_rates_setpoint
/fmu/in/vehicle_command
```

This demo keeps PX4 responsible for rate-loop execution, mixer/control allocation,
motor limits, arming, mode switching, and Offboard-loss behavior. The NMPC only
computes body roll/pitch/yaw rates plus normalized body-FRD thrust.

### NMPC Problem Formulation

The runtime solve chain is:

```text
/fmu/out/vehicle_odometry
  -> body_rate_nmpc_controller.py
  -> acados generated C solver
  -> /controller/body_rate/output
  -> fsm_node
  -> /fmu/in/vehicle_rates_setpoint
```

The NMPC state is:

```text
x = [
  p_n, p_e, p_d,
  v_n, v_e, v_d,
  q_w, q_x, q_y, q_z
]
```

This is the PX4 NED position, NED velocity, and Hamilton quaternion from body-FRD
to NED.

The NMPC control input is:

```text
u = [
  roll_rate,
  pitch_rate,
  yaw_rate,
  thrust
]
```

The ROS output maps this to PX4 as:

```text
VehicleRatesSetpoint.roll = roll_rate
VehicleRatesSetpoint.pitch = pitch_rate
VehicleRatesSetpoint.yaw = yaw_rate
VehicleRatesSetpoint.thrust_body = [0, 0, -thrust]
```

For PX4 multicopters, upward body-FRD thrust is represented by a negative
`thrust_body[2]`.

The simplified model is:

```text
p_dot = v
v_dot = g_ned + R(q) * [0, 0, -mass * g * thrust / hover_thrust] / mass
q_dot = 0.5 * q * [0, roll_rate, pitch_rate, yaw_rate]
```

The demo does not model motor lag, drag, torque allocation, or full rotational
dynamics. That is intentional for a BODY_RATE + THRUST controller: PX4 still runs
the rate controller, mixer/control allocation, and actuator limits.

The OCP is:

```text
min sum_k || [x_k, u_k] - [x_ref, u_ref] ||_W^2
  + || x_N - x_ref ||_W_e^2
```

Subject to:

```text
x_0 = current odometry state

roll_rate  in [-3.0, 3.0] rad/s
pitch_rate in [-3.0, 3.0] rad/s
yaw_rate   in [-1.5, 1.5] rad/s
thrust     in [0.10, 0.90]
```

Current solver settings:

```text
horizon steps: 20
horizon time: 1.0 s
dt: 0.05 s
QP solver: PARTIAL_CONDENSING_HPIPM
NLP solver: SQP_RTI
integrator: ERK
Hessian approximation: GAUSS_NEWTON
```

The demo reference is a square waypoint list:

```text
[0, 0, -2]
[3, 0, -2]
[3, 3, -2]
[0, 3, -2]
```

Velocity reference is zero and yaw reference is zero. The controller switches to
the next point when the current target is within `target_acceptance_radius_m`.

Each control tick:

1. Read the latest `VehicleOdometry` as `x0`.
2. Build `x_ref` from the active target point and yaw reference.
3. Set the first-stage equality constraint `lbx = ubx = x0`.
4. Set all stage references `yref = [x_ref, u_ref]`.
5. Solve with acados.
6. Read the first control `u0`.
7. Publish `u0` as `VehicleRatesSetpoint` on `/controller/body_rate/output`.

## Pure Thrust RL Controller

`rl_thrust_controller` is a minimal ONNX Runtime C++ inference node for policies
that output normalized thrust only. It is intentionally kept behind the same FSM
and adapter path as the other controllers:

```text
/fmu/out/vehicle_odometry
  -> rl_thrust_controller
  -> /controller/thrust/output
  -> fsm_node
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/vehicle_thrust_setpoint
  -> /fmu/in/vehicle_torque_setpoint
```

The node subscribes to:

```text
/fmu/out/vehicle_odometry
```

The node publishes:

```text
/controller/thrust/output
```

The FSM adapter publishes to PX4:

```text
/fmu/in/offboard_control_mode
/fmu/in/vehicle_thrust_setpoint
/fmu/in/vehicle_torque_setpoint
/fmu/in/vehicle_command
```

The default policy input is a float32 tensor with shape `[1, 10]`:

```text
[
  p_n, p_e, p_d,
  v_n, v_e, v_d,
  q_w, q_x, q_y, q_z
]
```

The policy output can be either:

- `[1]`: scalar normalized thrust. The node clamps it to `thrust_min` and
  `thrust_max`, then writes it to `VehicleThrustSetpoint.xyz[thrust_axis]` with
  `thrust_sign`.
- `[3]`: body-FRD thrust vector. The node clamps each element to `[-1, 1]` and
  publishes it directly.

Default scalar mapping:

```text
thrust_axis = 2
thrust_sign = -1.0
VehicleThrustSetpoint.xyz = [0, 0, -thrust]
```

Start PX4 SITL and the DDS bridge first. Then launch:

```bash
cd /home/li/Desktop/ws_ros2
source install/setup.bash
ros2 launch px4_ros2_ctrl fsm_rl_thrust.launch.py model_path:=/absolute/path/to/policy.onnx
```

Request Offboard after PX4 and estimator data are healthy:

```bash
ros2 service call /fsm_node/start_offboard std_srvs/srv/Trigger
```

Useful parameters:

```bash
ros2 launch px4_ros2_ctrl fsm_rl_thrust.launch.py \
  model_path:=/absolute/path/to/policy.onnx \
  thrust_min:=0.0 \
  thrust_max:=0.9 \
  thrust_axis:=2 \
  thrust_sign:=-1.0
```

Safety note: a pure thrust controller does not control attitude, yaw, or body
rates. For PX4 Offboard `thrust_and_torque`, this package publishes zero torque
setpoints together with the thrust setpoint. That is useful for interface testing,
vertical thrust experiments, or policies designed around another stabilizing layer,
but it is not a complete multicopter flight controller by itself. Do not use this
path on real hardware until the attitude/torque stabilization strategy is explicit
and validated in SITL.

## Adding A New Controller

Controllers should not publish `/fmu/in/*` topics directly. A controller should:

1. Subscribe to PX4/estimator state topics it needs.
2. Compute its control output.
3. Publish output to a controller namespace topic.
4. Let `fsm_node` and `Px4OutputAdapter` decide whether and how that output reaches
   PX4.

Currently implemented output path:

- POSITION output via `px4_msgs/msg/TrajectorySetpoint`
- Topic: `/controller/position/output`
- PX4 adapter output:
  - `/fmu/in/offboard_control_mode` with `position=true`
  - `/fmu/in/trajectory_setpoint`
- BODY_RATE + thrust output via `px4_msgs/msg/VehicleRatesSetpoint`
- Topic: `/controller/body_rate/output`
- PX4 adapter output:
  - `/fmu/in/offboard_control_mode` with `body_rate=true`
  - `/fmu/in/vehicle_rates_setpoint`
- Pure thrust output via `px4_msgs/msg/VehicleThrustSetpoint`
- Topic: `/controller/thrust/output`
- PX4 adapter output:
  - `/fmu/in/offboard_control_mode` with `thrust_and_torque=true`
  - `/fmu/in/vehicle_thrust_setpoint`
  - `/fmu/in/vehicle_torque_setpoint` with zero torque

Planned extension points:

- MPC velocity/acceleration output: `ControlLevel::VELOCITY` or
  `ControlLevel::ACCELERATION`
- SO3 attitude output: `ControlLevel::ATTITUDE`
- direct actuator output: `ControlLevel::ACTUATOR`

## Thrust Calibration Recorder

The package provides a ROS 2 C++ equivalent of the legacy `thrust_calibrate.py`
recorder:

```bash
ros2 run px4_ros2_ctrl thrust_calibration_node
```

This node does not command the vehicle. It records battery voltage and the currently
published thrust command, averages them over a configurable time window, and appends
the result to a CSV file.

Default inputs:

- Battery voltage: `/fmu/out/battery_status`
- Thrust command: `/fmu/in/vehicle_attitude_setpoint`
- Compatibility trigger: `/traj_start_trigger`

Start recording:

```bash
ros2 service call /thrust_calibration_node/start std_srvs/srv/Trigger
```

Stop and save:

```bash
ros2 service call /thrust_calibration_node/stop std_srvs/srv/Trigger
```

Clear buffered data:

```bash
ros2 service call /thrust_calibration_node/reset std_srvs/srv/Trigger
```

Useful parameters:

```bash
ros2 run px4_ros2_ctrl thrust_calibration_node --ros-args \
  -p output_file:=/tmp/thrust_calibration.csv \
  -p time_interval:=1.0 \
  -p min_battery_voltage:=13.2 \
  -p mass_kg:=1.0
```

For multicopters using `VehicleAttitudeSetpoint`, PX4 stores normalized thrust in
`thrust_body[2]` and the command is usually negative in body FRD. The recorder uses
`thrust_axis:=2` and `thrust_sign:=-1.0` by default so saved commands are positive.

If your controller publishes `VehicleThrustSetpoint` instead, run:

```bash
ros2 run px4_ros2_ctrl thrust_calibration_node --ros-args \
  -p use_thrust_setpoint_topic:=true
```

Fit voltage compensation parameters from the recorded CSV:

```bash
ros2 run px4_ros2_ctrl fit_thrust_calibration.py /tmp/thrust_calibration.csv
```

The default fitted model is:

```text
command = hover_command * (nominal_voltage / voltage) ^ exponent
```

The script prints JSON with:

- `hover_command`: normalized thrust command at `nominal_voltage`
- `voltage_exponent`: voltage compensation strength
- `newtons_per_normalized_command_at_nominal_voltage`: approximate force mapping
  if `mass_kg` is available
- `rmse` and `r2`: fit quality indicators

This fit assumes the recorded samples are hover or near-hover points, so average
required thrust is approximately `mass * 9.80665`. For a full dynamic thrust model,
record acceleration as well as voltage and command.

## Notes For Real Flight

Validate in SITL before real hardware. For real flight, verify:

- RC link is active and mapped to a manual/position mode switch.
- PX4 Offboard-loss parameters are configured intentionally.
- `allow_auto_arm` is false until you have a tested arming procedure.
- Controller outputs are bounded and expressed in PX4 NED conventions.
- Manual takeover causes the FSM to enter `MANUAL_OVERRIDE` and not re-enter
  Offboard without an explicit reset and start request.
