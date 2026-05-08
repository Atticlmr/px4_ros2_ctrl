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
- Auto-arm is disabled by default. Set `allow_auto_arm:=true` only after bench and
  SITL validation.
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

Planned extension points:

- MPC velocity/acceleration output: `ControlLevel::VELOCITY` or
  `ControlLevel::ACCELERATION`
- SO3 attitude output: `ControlLevel::ATTITUDE`
- body-rate controller output: `ControlLevel::BODY_RATE`

When adding ATTITUDE or BODY_RATE control, also extend `Px4OutputAdapter` to publish
the matching PX4 message type, such as `VehicleAttitudeSetpoint` or
`VehicleRatesSetpoint`.

## Notes For Real Flight

Validate in SITL before real hardware. For real flight, verify:

- RC link is active and mapped to a manual/position mode switch.
- PX4 Offboard-loss parameters are configured intentionally.
- `allow_auto_arm` is false until you have a tested arming procedure.
- Controller outputs are bounded and expressed in PX4 NED conventions.
- Manual takeover causes the FSM to enter `MANUAL_OVERRIDE` and not re-enter
  Offboard without an explicit reset and start request.
