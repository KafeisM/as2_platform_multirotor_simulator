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
