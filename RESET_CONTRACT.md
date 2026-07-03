# ResetSimulatorState Contract

This package provides `/platform/reset_simulator_state` as the simulator-owned
episode reset path for the AS2 Multirotor Simulator overlay used by
`rl_uav_aerostack2`.

## Request contract

The request targets the current node namespace only. There is no namespace field
inside `ResetSimulatorState`; callers select the vehicle by calling the service
under `/droneN/platform/reset_simulator_state`.

Required request fields:

- `x`, `y`, `z`, `yaw`: target world-frame pose.
- `position_tolerance`, `yaw_tolerance`: maximum accepted pose/yaw error after
  mutation.
- `linear_speed_tolerance`, `angular_speed_tolerance`: maximum accepted velocity
  norms after mutation.

The implementation rejects non-finite pose values, target altitudes below the
simulator floor, non-finite tolerances, and non-positive tolerances.

## Internal reset proof

`src/as2_platform_multirotor_simulator.cpp` makes reset proof concrete by
mutating and reporting the following state:

| Artifact | Implementation proof | Response proof |
|---|---|---|
| Position/orientation | `reset_state.kinematics.position`, `reset_state.kinematics.orientation` | `position_error`, `yaw_error` |
| Linear/angular velocity | `reset_state.kinematics.linear_velocity`, `reset_state.kinematics.angular_velocity` | `linear_speed_norm`, `angular_speed_norm` |
| Linear/angular acceleration | `reset_state.kinematics.linear_acceleration`, `reset_state.kinematics.angular_acceleration` | `linear_acceleration_norm`, `angular_acceleration_norm` |
| Force/torque | `reset_state.dynamics.force`, `reset_state.dynamics.torque` | `force_norm`, `torque_norm` |
| Motor state | `reset_state.actuators.motor_angular_velocity`, `reset_state.actuators.motor_angular_acceleration` | `motor_angular_velocity_norm`, `motor_angular_acceleration_norm`, `motor_state_reset_applied` |
| Dynamics mutation | `simulator_.get_dynamics().set_state(reset_state)` | `dynamics_reset_applied` |
| Controller | `simulator_.get_controller().reset_controller()` | `controller_reset_applied` |
| IMU | `simulator_.get_imu().reset()` | `imu_reset_applied` |
| Odometry | `simulator_.get_inertial_odometry().set_initial_position(...)`, `set_initial_orientation(...)`, `get_inertial_odometry().reset()` | `odometry_reset_applied` |
| References | `set_reference_position`, `set_reference_velocity`, `set_reference_trajectory`, `set_reference_yaw_angle`, `set_reference_yaw_rate`, `set_reference_acro`, zero motor reference | `references_reset_applied` |

`dynamic_artifacts_cleared` is true only when all reported acceleration, force,
torque, and motor norms are at or below the internal artifact tolerance and all
reset proof flags were applied.

## AS2 platform state machine synchronization

The low-level reset alone leaves the AS2 `AerialPlatform` layer untouched, and
the base-class `sendCommand()` gate (`connected && armed && offboard && control
mode settled && FSM not EMERGENCY`) then silently drops every motion reference:
commands are accepted on the ROS side but never forwarded to the simulator.

After mutating the simulator state, the reset callback therefore synchronizes
the platform layer:

| Artifact | Implementation proof | Response proof |
|---|---|---|
| Connected + FSM baseline | `resetPlatform()` (connected=true, FSM forced to DISARMED; the only sanctioned recovery from EMERGENCY) | `platform_fsm_synced` |
| Armed | `setArmingState(true)` (arms simulator, FSM DISARMED -> LANDED) | `platform_fsm_synced` |
| Offboard | `setOffboardControl(true)` | `platform_fsm_synced` |
| FSM FLYING | `handleStateMachineEvent(TAKE_OFF)` then `handleStateMachineEvent(TOOK_OFF)` (LANDED -> TAKING_OFF -> FLYING) | `platform_fsm_synced` |
| Control mode mirror | `setPlatformControlMode(POSITION / YAW_ANGLE / LOCAL_ENU_FRAME)` mirrors the forced simulator mode into `platform_info_msg_` so the next `SetControlMode` request (e.g. SPEED) is applied instead of skipped as already settled | `platform_control_mode_synced` |

`platform_fsm_synced` is true only when every FSM transition in the sequence
succeeded. Both flags are part of the `success` conjunction: a reset that
cannot leave the platform armed, offboard, FLYING, and with a consistent
control mode is reported as failed.

## Timer dt re-baseline on reset

The simulator update timers (dynamics, controller, inertial odometry) compute
their integration step from wall-clock time. The last-update timestamps are
stored as node members (`last_time_dynamics_`, `last_time_control_`,
`last_time_inertial_odometry_`) instead of function-local statics so the reset
callback can re-baseline them.

`resetSimulatorStateCallback` calls `rebaselineTimerBookkeeping()` right after
mutating the simulator state, setting all three timestamps to `this->now()`.
The first post-reset step therefore integrates roughly one nominal timer tick
instead of the whole gap spanned by the reset service handling. The timestamps
are also baselined at construction, so the very first timer fire integrates
only the construction-to-spin gap. The existing `dt <= 0` guards are kept and
extended with a `std::isfinite(dt)` check.

## Non-finite state guard on telemetry publishing

`simulatorStateTimerCallback` validates the simulator state before publishing:
ground-truth kinematics, inertial-odometry kinematics, and the IMU measurement
must contain only finite values (position, orientation, velocities and
accelerations). The vendored floor-collision clamp cannot catch NaN
(`position.z() <= floor_height` is false for NaN), so without this guard a NaN
would escape to IMU/Odom/GroundTruth/GPS/TF and crash RViz and downstream
consumers.

When a non-finite value is detected the callback:

1. Logs a throttled error:
   `Non-finite simulator state detected, skipping state publish and restoring
   last known finite state`.
2. Skips every publish for that cycle (IMU, odometry, ground truth, GPS, TF,
   gimbal).
3. Recovers the simulator instead of editing the vendored library: it restores
   the dynamics state from `last_finite_state_` (the last fully finite state
   snapshot, updated on every healthy publish cycle, at construction, and set
   to the reset pose on every reset), resets the controller, the IMU, and
   re-seeds and resets the inertial odometry from that snapshot pose.

This is the least invasive recovery that keeps the vendored headers untouched:
the simulator resumes from the newest provably finite state, which after a
reset is the reset pose itself.

## Defensive dt guard at integration call sites

The vendored `multirotor_dynamic_model/dynamics.hpp` throws
`std::invalid_argument` when `dt <= 0` reaches `update_dynamics`; an uncaught
throw from a timer callback would `std::terminate` the node. All platform-side
integration entry points (`update_dynamics`/`update_imu`,
`update_controller`, `update_inertial_odometry`) live in the three timer
callbacks, which `ownTakeoff`/`ownLand` also reuse for their manual spin
loops. Each callback drops the tick when `dt <= 0` or non-finite, and the
integration calls are additionally wrapped in a `try/catch` that logs a
throttled `Skipping ... update step (dt=...)` error and skips the tick rather
than letting the exception kill the node.

## Telemetry-side proof boundary

The service response proves internal state mutation at the immediate reset
boundary. It does not prove that the next sampled ROS telemetry stream is free of
transient artifacts after timers advance. Live validation must still sample `/imu`,
`/ground_truth/pose`, `/ground_truth/twist`, and odometry after reset to prove the
telemetry-side proof boundary: no post-reset teleport spike, near-zero twist, and
near-zero acceleration where observable.

## Out-of-scope work

ROS-free Python bindings and fixed-dt simulator APIs are future work only. They
must not be used as acceptance evidence for this reset contract, which targets the
ROS-integrated AS2 Multirotor Simulator overlay.
