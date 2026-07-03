// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file as2_platform_multirotor_simulator.cpp
 *
 * MultirotorSimulatorPlatform class implementation
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "as2_platform_multirotor_simulator/as2_platform_multirotor_simulator.hpp"

#include <cmath>

#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"

namespace as2_platform_multirotor_simulator
{

MultirotorSimulatorPlatform::MultirotorSimulatorPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options), as2_interface_(this)
{
  RCLCPP_INFO(this->get_logger(), "Initializing MultirotorSimulatorPlatform...");

  // Read parameters
  readParams(platform_params_);

  // Baseline the timers dt bookkeeping and the last known finite state
  rebaselineTimerBookkeeping();
  last_finite_state_ = simulator_.get_state();

  // Timers
  RCLCPP_INFO(this->get_logger(), "Using update freq: %f", platform_params_.update_freq);
  RCLCPP_INFO(this->get_logger(), "Using control freq: %f", platform_params_.control_freq);
  RCLCPP_INFO(
    this->get_logger(), "Using inertial odometry freq: %f",
    platform_params_.inertial_odometry_freq);
  RCLCPP_INFO(this->get_logger(), "Using state pub freq: %f", platform_params_.state_freq);
  RCLCPP_INFO(
    this->get_logger(), "GPS origin: %f, %f, %f", platform_params_.latitude,
    platform_params_.longitude, platform_params_.altitude);

  simulator_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / platform_params_.update_freq),
    std::bind(&MultirotorSimulatorPlatform::simulatorTimerCallback, this));
  simulator_control_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / platform_params_.control_freq),
    std::bind(&MultirotorSimulatorPlatform::simulatorControlTimerCallback, this));
  simulator_inertial_odometry_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / platform_params_.inertial_odometry_freq),
    std::bind(&MultirotorSimulatorPlatform::simulatorInertialOdometryTimerCallback, this));
  simulator_state_pub_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / platform_params_.imu_pub_freq),
    std::bind(&MultirotorSimulatorPlatform::simulatorStateTimerCallback, this));

  gps_handler_.setOrigin(
    platform_params_.latitude, platform_params_.longitude,
    platform_params_.altitude);  // Set origin for GPS

  // Configure sensors
  configureSensors();

  reset_simulator_state_srv_ = this->create_service<ResetSimulatorState>(
    "platform/reset_simulator_state",
    std::bind(
      &MultirotorSimulatorPlatform::resetSimulatorStateCallback, this,
      std::placeholders::_1, std::placeholders::_2));
}

MultirotorSimulatorPlatform::~MultirotorSimulatorPlatform()
{
  // Timers
  simulator_timer_.reset();
  simulator_control_timer_.reset();
  simulator_state_pub_timer_.reset();

  // Sensors
  sensor_ground_truth_ptr_.reset();
  sensor_odom_estimate_ptr_.reset();
  sensor_imu_ptr_.reset();
  sensor_gps_ptr_.reset();
}

void MultirotorSimulatorPlatform::configureSensors()
{
  getParam("global_ref_frame", frame_id_earth_);
  getParam("base_frame", frame_id_baselink_);
  frame_id_baselink_ = as2::tf::generateTfName(this, frame_id_baselink_);

  // Get gimbal name
  std::string gimbal_name = "gimbal";
  std::string gimbal_base_name = "gimbal_base";
  getParam("gimbal.frame_id", gimbal_name);
  getParam("gimbal.base_frame_id", gimbal_base_name);

  RCLCPP_INFO(this->get_logger(), "Ground truth freq: %f", platform_params_.ground_truth_pub_freq);
  RCLCPP_INFO(this->get_logger(), "Odometry freq: %f", platform_params_.odometry_pub_freq);
  RCLCPP_INFO(this->get_logger(), "IMU freq: %f", platform_params_.imu_pub_freq);
  RCLCPP_INFO(this->get_logger(), "GPS freq: %f", platform_params_.gps_pub_freq);

  // Ground truth
  sensor_ground_truth_ptr_ = std::make_unique<as2::sensors::GroundTruth>(
    this, platform_params_.ground_truth_pub_freq);
  // Odometry
  sensor_odom_estimate_ptr_ = std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>(
    as2_names::topics::sensor_measurements::odom, this, platform_params_.odometry_pub_freq);
  // IMU
  sensor_imu_ptr_ = std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::Imu>>(
    as2_names::topics::sensor_measurements::imu, this, platform_params_.imu_pub_freq);
  // GPS
  sensor_gps_ptr_ = std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::NavSatFix>>(
    as2_names::topics::sensor_measurements::gps, this, platform_params_.gps_pub_freq);
  // Gimbal
  sensor_gimbal_ptr_ = std::make_unique<as2::sensors::Gimbal>(
    gimbal_name, gimbal_base_name, this, platform_params_.gimbal_pub_freq);
  gimbal_control_sub_ = this->create_subscription<as2_msgs::msg::GimbalControl>(
    "platform/" + gimbal_name + "/gimbal_command", 10,
    std::bind(&MultirotorSimulatorPlatform::gimbalControlCallback, this, std::placeholders::_1));

  geometry_msgs::msg::Transform gimbal_transform;
  getParam("gimbal.base_transform.x", gimbal_transform.translation.x);
  getParam("gimbal.base_transform.y", gimbal_transform.translation.y);
  getParam("gimbal.base_transform.z", gimbal_transform.translation.z);
  sensor_gimbal_ptr_->setGimbalBaseTransform(gimbal_transform);
}

bool MultirotorSimulatorPlatform::ownSetArmingState(bool state)
{
  state ? simulator_.arm() : simulator_.disarm();
  RCLCPP_INFO(this->get_logger(), "Arming state set to %d.", state);
  return true;
}

bool MultirotorSimulatorPlatform::ownSetOffboardControl(bool offboard)
{
  RCLCPP_INFO(this->get_logger(), "Offboard state set to %d.", offboard);
  return true;
}

bool MultirotorSimulatorPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  if (platform_info_msg_.current_control_mode.control_mode == msg.control_mode) {
    RCLCPP_INFO(
      this->get_logger(), "Control mode already set to [%s]",
      as2::control_mode::controlModeToString(msg).c_str());
    return true;
  }

  multirotor::YawControlMode yaw_mode = multirotor::YawControlMode::ANGLE;
  if (msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    yaw_mode = multirotor::YawControlMode::RATE;
  }

  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
    case as2_msgs::msg::ControlMode::HOVER:
      {
        simulator_.set_control_mode(multirotor::ControlMode::HOVER);
        break;
      }
    case as2_msgs::msg::ControlMode::POSITION:
      {
        simulator_.set_control_mode(multirotor::ControlMode::POSITION, yaw_mode);
        break;
      }
    case as2_msgs::msg::ControlMode::SPEED:
      {
        simulator_.set_control_mode(multirotor::ControlMode::VELOCITY, yaw_mode);
        break;
      }
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      {
        simulator_.set_control_mode(multirotor::ControlMode::TRAJECTORY, yaw_mode);
        break;
      }
    case as2_msgs::msg::ControlMode::ACRO:
      {
        simulator_.set_control_mode(multirotor::ControlMode::ACRO);
        break;
      }
    default:
      {
        RCLCPP_ERROR(
          this->get_logger(), "Desired control mode not supported: [%s]",
          as2::control_mode::controlModeToString(msg).c_str());
        return false;
        break;
      }
  }
  RCLCPP_INFO(
    this->get_logger(), "Control mode set to [%s]", as2::control_mode::controlModeToString(
      msg).c_str());
  return true;
}

bool MultirotorSimulatorPlatform::ownSendCommand()
{
  const as2_msgs::msg::ControlMode current_control_mode = platform_info_msg_.current_control_mode;

  switch (current_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
    case as2_msgs::msg::ControlMode::HOVER:
      {
        // Hovering
        break;
      }
    case as2_msgs::msg::ControlMode::POSITION:
      {
        // If not using odom for control, convert to earth frame
        if (!using_odom_for_control_) {
          if (!as2_interface_.processCommand(command_pose_msg_) || !as2_interface_.processCommand(
              command_twist_msg_))
          {
            return false;
          }
        }
        RCLCPP_INFO(
          this->get_logger(), "Setting position to: [%f, %f, %f]",
          command_pose_msg_.pose.position.x,
          command_pose_msg_.pose.position.y, command_pose_msg_.pose.position.z);
        Eigen::Vector3d position;
        position.x() = command_pose_msg_.pose.position.x;
        position.y() = command_pose_msg_.pose.position.y;
        position.z() = command_pose_msg_.pose.position.z;
        simulator_.set_reference_position(position);

        // Velocity limits
        Eigen::Vector3d velocity;
        velocity.x() = command_twist_msg_.twist.linear.x;
        velocity.y() = command_twist_msg_.twist.linear.y;
        velocity.z() = command_twist_msg_.twist.linear.z;
        if (velocity.norm() > 0.0) {
          const bool proportional_saturation_flag =
            simulator_.get_controller_const().get_position_controller_const().
            get_proportional_saturation_flag();
          simulator_.get_controller().get_position_controller().set_output_saturation(
            velocity,
            -velocity,
            proportional_saturation_flag);
        }
        break;
      }
    case as2_msgs::msg::ControlMode::SPEED:
      {
        // If not using odom for control, convert to earth frame
        if (!using_odom_for_control_) {
          if (!as2_interface_.processCommand(command_twist_msg_)) {
            return false;
          }
        }
        Eigen::Vector3d velocity;
        velocity.x() = command_twist_msg_.twist.linear.x;
        velocity.y() = command_twist_msg_.twist.linear.y;
        velocity.z() = command_twist_msg_.twist.linear.z;
        simulator_.set_reference_velocity(velocity);
        break;
      }
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      {
        // If not using odom for control, convert to earth frame
        if (!using_odom_for_control_) {
          if (!as2_interface_.processCommand(command_trajectory_msg_)) {
            return false;
          }
        }
        Eigen::Vector3d position, velocity, acceleration;
        position.x() = command_trajectory_msg_.setpoints[0].position.x;
        position.y() = command_trajectory_msg_.setpoints[0].position.y;
        position.z() = command_trajectory_msg_.setpoints[0].position.z;
        velocity.x() = command_trajectory_msg_.setpoints[0].twist.x;
        velocity.y() = command_trajectory_msg_.setpoints[0].twist.y;
        velocity.z() = command_trajectory_msg_.setpoints[0].twist.z;
        acceleration.x() = command_trajectory_msg_.setpoints[0].acceleration.x;
        acceleration.y() = command_trajectory_msg_.setpoints[0].acceleration.y;
        acceleration.z() = command_trajectory_msg_.setpoints[0].acceleration.z;

        simulator_.set_reference_trajectory(
          position, velocity, acceleration);
        break;
      }
    case as2_msgs::msg::ControlMode::ACRO:
      {
        double thrust = command_thrust_msg_.thrust;
        Eigen::Vector3d angular_velocity;
        angular_velocity.x() = command_twist_msg_.twist.angular.x;
        angular_velocity.y() = command_twist_msg_.twist.angular.y;
        angular_velocity.z() = command_twist_msg_.twist.angular.z;

        simulator_.set_reference_acro(thrust, angular_velocity);
        break;
      }
    default:
      RCLCPP_ERROR(
        this->get_logger(), "Control mode %d not supported.",
        platform_info_msg_.current_control_mode.control_mode);
      return false;
      break;
  }

  switch (current_control_mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      {
        simulator_.set_reference_yaw_rate(command_twist_msg_.twist.angular.z);
        break;
      }
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
    default:
      {
        double roll, pitch, yaw;
        as2::frame::quaternionToEuler(command_pose_msg_.pose.orientation, roll, pitch, yaw);
        if (current_control_mode.control_mode ==
          as2_msgs::msg::ControlMode::TRAJECTORY)
        {
          yaw = command_trajectory_msg_.setpoints[0].yaw_angle;
        } else {
          as2::frame::quaternionToEuler(command_pose_msg_.pose.orientation, roll, pitch, yaw);
        }
        simulator_.set_reference_yaw_angle(yaw);
        break;
      }
  }

  return true;
}

void MultirotorSimulatorPlatform::ownStopPlatform()
{
  // Send hover to platform here
  as2_msgs::msg::ControlMode control_mode_msg;
  control_mode_msg.control_mode = as2_msgs::msg::ControlMode::HOVER;
  setPlatformControlMode(control_mode_msg);
}

void MultirotorSimulatorPlatform::ownKillSwitch()
{
  // Switch off motors
  simulator_.disarm();
}

bool MultirotorSimulatorPlatform::ownTakeoff()
{
  // Send takeoff to platform here

  // Set control mode to position
  as2_msgs::msg::ControlMode control_mode_msg;
  control_mode_msg.control_mode = as2_msgs::msg::ControlMode::POSITION;
  control_mode_msg.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  control_mode_msg.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  setPlatformControlMode(control_mode_msg);

  // Set reference position to current position and 1m above
  command_pose_msg_.header.frame_id = frame_id_earth_;
  command_pose_msg_.header.stamp = this->now();
  command_pose_msg_.pose.position.x = simulator_.get_state().kinematics.position.x();
  command_pose_msg_.pose.position.y = simulator_.get_state().kinematics.position.y();
  const double takeoff_height = simulator_.get_floor_height() + 1.0;
  command_pose_msg_.pose.position.z = takeoff_height;
  command_pose_msg_.pose.orientation.w = simulator_.get_state().kinematics.orientation.w();
  command_pose_msg_.pose.orientation.x = simulator_.get_state().kinematics.orientation.x();
  command_pose_msg_.pose.orientation.y = simulator_.get_state().kinematics.orientation.y();
  command_pose_msg_.pose.orientation.z = simulator_.get_state().kinematics.orientation.z();

  // Set reference velocity to 1m/s to speed limit
  command_twist_msg_.header.frame_id = frame_id_earth_;
  command_twist_msg_.header.stamp = this->now();
  command_twist_msg_.twist.linear.x = 1.0;
  command_twist_msg_.twist.linear.y = 1.0;
  command_twist_msg_.twist.linear.z = 1.0;

  // Set references
  if (!ownSendCommand()) {
    return false;
  }

  // TODO(RPS98): Use multithread execution
  while (rclcpp::ok() &&
    std::abs(simulator_.get_state().kinematics.position.z() - takeoff_height) > 0.2)
  {
    // Spin timers
    simulatorTimerCallback();
    simulatorControlTimerCallback();
    simulatorInertialOdometryTimerCallback();
    simulatorStateTimerCallback();
  }
  return true;
}

bool MultirotorSimulatorPlatform::ownLand()
{
  // Send land to platform here

  // Set control mode to position
  as2_msgs::msg::ControlMode control_mode_msg;
  control_mode_msg.control_mode = as2_msgs::msg::ControlMode::POSITION;
  control_mode_msg.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  control_mode_msg.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  setPlatformControlMode(control_mode_msg);

  // Set reference position to current position and 1m above
  command_pose_msg_.header.frame_id = frame_id_earth_;
  command_pose_msg_.header.stamp = this->now();
  command_pose_msg_.pose.position.x = simulator_.get_state().kinematics.position.x();
  command_pose_msg_.pose.position.y = simulator_.get_state().kinematics.position.y();
  const double land_height = simulator_.get_floor_height();
  command_pose_msg_.pose.position.z = land_height;
  command_pose_msg_.pose.orientation.w = simulator_.get_state().kinematics.orientation.w();
  command_pose_msg_.pose.orientation.x = simulator_.get_state().kinematics.orientation.x();
  command_pose_msg_.pose.orientation.y = simulator_.get_state().kinematics.orientation.y();
  command_pose_msg_.pose.orientation.z = simulator_.get_state().kinematics.orientation.z();

  // Set reference velocity to 1m/s to speed limit
  command_twist_msg_.header.frame_id = frame_id_earth_;
  command_twist_msg_.header.stamp = this->now();
  command_twist_msg_.twist.linear.x = 1.0;
  command_twist_msg_.twist.linear.y = 1.0;
  command_twist_msg_.twist.linear.z = 1.0;

  // Set references
  if (!ownSendCommand()) {
    return false;
  }

  // TODO(RPS98): Use multithread execution
  while (rclcpp::ok() &&
    std::abs(simulator_.get_state().kinematics.position.z() - land_height) > 0.2)
  {
    // Call timers
    simulatorTimerCallback();
    simulatorControlTimerCallback();
    simulatorInertialOdometryTimerCallback();
    simulatorStateTimerCallback();
  }
  return true;
}

void MultirotorSimulatorPlatform::gimbalControlCallback(
  const as2_msgs::msg::GimbalControl::SharedPtr msg)
{
  double roll, pitch, yaw;
  switch (msg->control_mode) {
    case as2_msgs::msg::GimbalControl::POSITION_MODE:
      {
        roll = msg->target.vector.x;
        pitch = msg->target.vector.y;
        yaw = msg->target.vector.z;
        break;
      }
    case as2_msgs::msg::GimbalControl::SPEED_MODE:
    // TODO(RPS98): Implement speed mode
    // {
    //   as2::frame::quaternionToEuler(gimbal_desired_orientation_.quaternion, roll, pitch, yaw);
    //   double dt = 1.0 / platform_params_.gimbal_pub_freq;
    //   roll += msg->target.vector.x * dt;
    //   pitch += msg->target.vector.y * dt;
    //   yaw += msg->target.vector.z * dt;
    //   break;
    // }
    default:
      {
        RCLCPP_ERROR(
          this->get_logger(), "Gimbal control mode %d not supported.", msg->control_mode);
        break;
      }
  }
  as2::frame::eulerToQuaternion(
    roll, pitch, yaw, gimbal_desired_orientation_.quaternion);
}

void MultirotorSimulatorPlatform::resetSimulatorStateCallback(
  const std::shared_ptr<ResetSimulatorState::Request> request,
  std::shared_ptr<ResetSimulatorState::Response> response)
{
  const auto reject = [&response](const std::string & message) {
      response->success = false;
      response->message = message;
    };

  if (!std::isfinite(request->x) || !std::isfinite(request->y) ||
    !std::isfinite(request->z) || !std::isfinite(request->yaw))
  {
    reject("reset target pose/yaw must contain only finite values");
    return;
  }

  if (request->z < simulator_.get_floor_height()) {
    reject("reset target z must not be below simulator floor height");
    return;
  }

  if (!std::isfinite(request->position_tolerance) || !std::isfinite(request->yaw_tolerance) ||
    !std::isfinite(request->linear_speed_tolerance) ||
    !std::isfinite(request->angular_speed_tolerance))
  {
    reject("reset tolerances must contain only finite values");
    return;
  }

  if (request->position_tolerance <= 0.0 || request->yaw_tolerance <= 0.0 ||
    request->linear_speed_tolerance <= 0.0 || request->angular_speed_tolerance <= 0.0)
  {
    reject("reset tolerances must be strictly positive");
    return;
  }

  const Eigen::Vector3d target_position(request->x, request->y, request->z);
  const Eigen::Vector3d zero_vector = Eigen::Vector3d::Zero();
  Eigen::Quaterniond target_orientation;
  as2::frame::eulerToQuaternion(0.0, 0.0, request->yaw, target_orientation);

  auto reset_state = simulator_.get_state();
  reset_state.kinematics.position = target_position;
  reset_state.kinematics.orientation = target_orientation;
  reset_state.kinematics.linear_velocity = zero_vector;
  reset_state.kinematics.angular_velocity = zero_vector;
  reset_state.kinematics.linear_acceleration = zero_vector;
  reset_state.kinematics.angular_acceleration = zero_vector;
  reset_state.dynamics.force = zero_vector;
  reset_state.dynamics.torque = zero_vector;
  reset_state.actuators.motor_angular_velocity.setZero();
  reset_state.actuators.motor_angular_acceleration.setZero();

  simulator_.get_dynamics().set_state(reset_state);
  simulator_.get_imu().reset();
  simulator_.get_inertial_odometry().set_initial_position(target_position);
  simulator_.get_inertial_odometry().set_initial_orientation(target_orientation);
  simulator_.get_inertial_odometry().reset();
  simulator_.get_controller().reset_controller();
  simulator_.arm();
  const bool dynamics_reset_applied = true;
  const bool motor_state_reset_applied = true;
  const bool imu_reset_applied = true;
  const bool odometry_reset_applied = true;
  const bool controller_reset_applied = true;

  control_state_ = reset_state.kinematics;

  // Re-baseline the timers dt bookkeeping so the first post-reset simulator
  // step integrates one nominal tick instead of the whole reset gap, and
  // track the reset state as the last known finite state for NaN recovery.
  rebaselineTimerBookkeeping();
  last_finite_state_ = reset_state;

  // Synchronize the AS2 platform layer with the post-reset flying state. The
  // base class sendCommand() gate requires connected + armed + offboard and a
  // settled control mode before forwarding commands to ownSendCommand();
  // without this sync, post-reset motion references are accepted on the ROS
  // side but never applied by the simulator.
  // resetPlatform() forces connected=true and FSM=DISARMED, which is also the
  // only sanctioned recovery from the EMERGENCY state.
  resetPlatform();
  bool platform_fsm_synced = setArmingState(true);  // FSM: DISARMED -> LANDED
  platform_fsm_synced = setOffboardControl(true) && platform_fsm_synced;
  platform_fsm_synced =
    handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::TAKE_OFF) &&
    platform_fsm_synced;  // FSM: LANDED -> TAKING_OFF
  platform_fsm_synced =
    handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::TOOK_OFF) &&
    platform_fsm_synced;  // FSM: TAKING_OFF -> FLYING
  if (!platform_fsm_synced) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Reset could not synchronize the AS2 platform state machine to FLYING");
  }

  // Mirror the forced simulator control mode (POSITION / YAW_ANGLE) into the
  // platform info so the next SetControlMode request (e.g. SPEED) is applied
  // to the simulator instead of being skipped as already settled.
  as2_msgs::msg::ControlMode reset_control_mode;
  reset_control_mode.control_mode = as2_msgs::msg::ControlMode::POSITION;
  reset_control_mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  reset_control_mode.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  const bool platform_control_mode_synced = setPlatformControlMode(reset_control_mode);

  // References must be applied AFTER the platform sync: setArmingState(true)
  // reaches simulator arm(), which clobbers reference_position_.z() to the
  // floor height and would otherwise make the position controller descend.
  auto zero_motors = simulator_.get_reference_motors_angular_velocity();
  zero_motors.setZero();
  simulator_.set_refence_motors_angular_velocity(zero_motors);
  simulator_.set_reference_position(target_position);
  simulator_.set_reference_velocity(zero_vector);
  simulator_.set_reference_trajectory(target_position, zero_vector, zero_vector);
  simulator_.set_reference_yaw_angle(request->yaw);
  simulator_.set_reference_yaw_rate(0.0);
  simulator_.set_reference_acro(0.0, zero_vector);
  simulator_.set_control_mode(multirotor::ControlMode::POSITION, multirotor::YawControlMode::ANGLE);
  const bool references_reset_applied = true;

  command_pose_msg_.header.frame_id = frame_id_earth_;
  command_pose_msg_.header.stamp = this->now();
  command_pose_msg_.pose.position.x = request->x;
  command_pose_msg_.pose.position.y = request->y;
  command_pose_msg_.pose.position.z = request->z;
  command_pose_msg_.pose.orientation.w = target_orientation.w();
  command_pose_msg_.pose.orientation.x = target_orientation.x();
  command_pose_msg_.pose.orientation.y = target_orientation.y();
  command_pose_msg_.pose.orientation.z = target_orientation.z();
  command_twist_msg_.header.frame_id = frame_id_earth_;
  command_twist_msg_.header.stamp = this->now();
  command_twist_msg_.twist.linear.x = 0.0;
  command_twist_msg_.twist.linear.y = 0.0;
  command_twist_msg_.twist.linear.z = 0.0;
  command_twist_msg_.twist.angular.x = 0.0;
  command_twist_msg_.twist.angular.y = 0.0;
  command_twist_msg_.twist.angular.z = 0.0;
  command_thrust_msg_.thrust = 0.0;
  command_trajectory_msg_.setpoints.clear();

  const auto current_simulator_state = simulator_.get_state();
  const auto current_state = current_simulator_state.kinematics;
  const double current_yaw = as2::frame::getYawFromQuaternion(current_state.orientation);
  const double yaw_error = std::atan2(
    std::sin(current_yaw - request->yaw), std::cos(current_yaw - request->yaw));

  response->position_error = (current_state.position - target_position).norm();
  response->yaw_error = std::abs(yaw_error);
  response->linear_speed_norm = current_state.linear_velocity.norm();
  response->angular_speed_norm = current_state.angular_velocity.norm();
  response->linear_acceleration_norm = current_state.linear_acceleration.norm();
  response->angular_acceleration_norm = current_state.angular_acceleration.norm();
  response->force_norm = current_simulator_state.dynamics.force.norm();
  response->torque_norm = current_simulator_state.dynamics.torque.norm();
  response->motor_angular_velocity_norm =
    current_simulator_state.actuators.motor_angular_velocity.norm();
  response->motor_angular_acceleration_norm =
    current_simulator_state.actuators.motor_angular_acceleration.norm();
  response->dynamics_reset_applied = dynamics_reset_applied;
  response->motor_state_reset_applied = motor_state_reset_applied;
  response->controller_reset_applied = controller_reset_applied;
  response->imu_reset_applied = imu_reset_applied;
  response->odometry_reset_applied = odometry_reset_applied;
  response->references_reset_applied = references_reset_applied;
  response->platform_fsm_synced = platform_fsm_synced;
  response->platform_control_mode_synced = platform_control_mode_synced;

  constexpr double artifact_tolerance = 1e-9;
  response->dynamic_artifacts_cleared =
    response->linear_acceleration_norm <= artifact_tolerance &&
    response->angular_acceleration_norm <= artifact_tolerance &&
    response->force_norm <= artifact_tolerance &&
    response->torque_norm <= artifact_tolerance &&
    response->motor_angular_velocity_norm <= artifact_tolerance &&
    response->motor_angular_acceleration_norm <= artifact_tolerance &&
    response->dynamics_reset_applied &&
    response->motor_state_reset_applied &&
    response->controller_reset_applied &&
    response->imu_reset_applied &&
    response->odometry_reset_applied &&
    response->references_reset_applied;

  response->success = response->position_error <= request->position_tolerance &&
    response->yaw_error <= request->yaw_tolerance &&
    response->linear_speed_norm <= request->linear_speed_tolerance &&
    response->angular_speed_norm <= request->angular_speed_tolerance &&
    response->dynamic_artifacts_cleared &&
    response->platform_fsm_synced &&
    response->platform_control_mode_synced;
  if (response->success) {
    response->message = "reset state applied";
  } else if (!response->platform_fsm_synced || !response->platform_control_mode_synced) {
    response->message = "platform FSM/control-mode sync failed after reset";
  } else {
    response->message = "reset state outside tolerance after mutation";
  }
}

Eigen::Vector3d MultirotorSimulatorPlatform::readVectorParams(const std::string & param_name)
{
  Eigen::Vector3d default_value = Eigen::Vector3d::Zero();  // Default value

  try {
    std::vector<double> vec;
    this->getParam(param_name, vec);

    if (vec.size() != 3) {
      RCLCPP_ERROR(
        this->get_logger(), "Parameter '%s' is not a vector of size 3.", param_name.c_str());
      // Print vector
      RCLCPP_ERROR(this->get_logger(), "Vector: ");
      for (auto & v : vec) {
        RCLCPP_ERROR(this->get_logger(), "%f", v);
      }
      return default_value;
    }

    return Eigen::Vector3d(vec[0], vec[1], vec[2]);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Error getting parameter %s: %s", param_name.c_str(), e.what());
    return default_value;
  }
}

void MultirotorSimulatorPlatform::readParams(PlatformParams & platform_params)
{
  RCLCPP_INFO(this->get_logger(), "Reading parameters...");
  // Platform Parameters
  getParam("imu_pub_freq", platform_params.imu_pub_freq);
  getParam("odometry_pub_freq", platform_params.odometry_pub_freq);
  getParam("ground_truth_pub_freq", platform_params.ground_truth_pub_freq);
  getParam("gps_pub_freq", platform_params.gps_pub_freq);
  getParam("gimbal.pub_freq", platform_params.gimbal_pub_freq);

  // Get max frequency
  platform_params.state_freq = std::max(
    std::max(platform_params.imu_pub_freq, platform_params.odometry_pub_freq),
    std::max(platform_params.ground_truth_pub_freq, platform_params.gps_pub_freq));

  // GPS Origin
  getParam("gps_origin.latitude", platform_params.latitude);
  getParam("gps_origin.longitude", platform_params.longitude);
  getParam("gps_origin.altitude", platform_params.altitude);

  // Simulator params
  double floor_height = 0.0;
  getParam("use_odom_for_control", using_odom_for_control_);
  getParam("floor_height", floor_height);
  getParam("simulation.update_freq", platform_params.update_freq);
  getParam("simulation.control_freq", platform_params.control_freq);
  getParam("simulation.inertial_odometry_freq", platform_params.inertial_odometry_freq);

  // Initial pose
  Eigen::Vector3d initial_position;
  getParam("vehicle_initial_pose.x", initial_position.x());
  getParam("vehicle_initial_pose.y", initial_position.y());
  getParam("vehicle_initial_pose.z", initial_position.z());
  double roll, pitch, yaw;
  getParam("vehicle_initial_pose.yaw", yaw);
  getParam("vehicle_initial_pose.pitch", pitch);
  getParam("vehicle_initial_pose.roll", roll);
  Eigen::Quaterniond initial_orientation;
  as2::frame::eulerToQuaternion(roll, pitch, yaw, initial_orientation);

  // Dynamics params
  // Dynamics::State params
  SimulatorParams::DynamicsParams & dp = simulator_params_.dynamics_params;
  dp.state.kinematics.position = initial_position;
  dp.state.kinematics.orientation = initial_orientation;

  // Dynamics::Model params
  dp.model_params.gravity = readVectorParams("multirotor.dynamics.model.gravity");
  getParam("multirotor.dynamics.model.vehicle_mass", dp.model_params.vehicle_mass);
  dp.model_params.vehicle_inertia =
    readVectorParams("multirotor.dynamics.model.vehicle_inertia").asDiagonal();
  getParam(
    "multirotor.dynamics.model.vehicle_drag_coefficient", dp.model_params.vehicle_drag_coefficient);
  dp.model_params.vehicle_aero_moment_coefficient =
    readVectorParams("multirotor.dynamics.model.vehicle_aero_moment_coefficient").asDiagonal();
  getParam(
    "multirotor.dynamics.model.force_process_noise_auto_correlation",
    dp.model_params.moment_process_noise_auto_correlation);
  getParam(
    "multirotor.dynamics.model.moment_process_noise_auto_correlation",
    dp.model_params.moment_process_noise_auto_correlation);

  double thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed,
    time_constant, rotational_inertia;
  getParam("multirotor.dynamics.model.motors_params.thrust_coefficient", thrust_coefficient);
  getParam("multirotor.dynamics.model.motors_params.torque_coefficient", torque_coefficient);
  getParam("multirotor.dynamics.model.motors_params.x_dist", x_dist);
  getParam("multirotor.dynamics.model.motors_params.y_dist", y_dist);
  getParam("multirotor.dynamics.model.motors_params.min_speed", min_speed);
  getParam("multirotor.dynamics.model.motors_params.max_speed", max_speed);
  getParam("multirotor.dynamics.model.motors_params.time_constant", time_constant);
  getParam("multirotor.dynamics.model.motors_params.rotational_inertia", rotational_inertia);
  dp.model_params.motors_params = multirotor::model::Model<double, 4>::create_quadrotor_x_config(
    thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed, time_constant,
    rotational_inertia);

  // Controller params Indi
  SimulatorParams::ControllerParams & cp = simulator_params_.controller_params;
  cp.indi_controller_params.inertia = dp.model_params.vehicle_inertia;
  auto mixing_matrix_6D_4rotors =
    multirotor::model::Model<double, 4>::compute_mixer_matrix<6>(dp.model_params.motors_params);
  cp.indi_controller_params.mixer_matrix_inverse =
    indi_controller::compute_quadrotor_mixer_matrix_inverse(mixing_matrix_6D_4rotors);

  cp.indi_controller_params.pid_params.Kp_gains =
    readVectorParams("multirotor.controller.indi.kp");
  cp.indi_controller_params.pid_params.Ki_gains =
    readVectorParams("multirotor.controller.indi.ki");
  cp.indi_controller_params.pid_params.Kd_gains =
    readVectorParams("multirotor.controller.indi.kd");
  cp.indi_controller_params.pid_params.alpha =
    readVectorParams("multirotor.controller.indi.alpha");
  cp.indi_controller_params.pid_params.antiwindup_cte =
    readVectorParams("multirotor.controller.indi.antiwindup_cte");
  Eigen::Vector3d angular_acceleration_limit =
    readVectorParams("multirotor.controller.indi.angular_acceleration_limit");
  cp.indi_controller_params.pid_params.upper_output_saturation = angular_acceleration_limit;
  cp.indi_controller_params.pid_params.lower_output_saturation = -angular_acceleration_limit;
  cp.indi_controller_params.pid_params.proportional_saturation_flag = true;

  // Controller params Acro
  cp.acro_controller_params.gravity = dp.model_params.gravity;
  cp.acro_controller_params.vehicle_mass = dp.model_params.vehicle_mass;
  cp.acro_controller_params.kp_rot =
    readVectorParams("multirotor.controller.acro.kp_rot").asDiagonal();

  // Controller params Trajectory
  cp.trajectory_controller_params.pid_params.Kp_gains =
    readVectorParams("multirotor.controller.trajectory.kp");
  cp.trajectory_controller_params.pid_params.Ki_gains =
    readVectorParams("multirotor.controller.trajectory.ki");
  cp.trajectory_controller_params.pid_params.Kd_gains =
    readVectorParams("multirotor.controller.trajectory.kd");
  cp.trajectory_controller_params.pid_params.alpha =
    readVectorParams("multirotor.controller.trajectory.alpha");
  cp.trajectory_controller_params.pid_params.antiwindup_cte =
    readVectorParams("multirotor.controller.trajectory.antiwindup_cte");
  Eigen::Vector3d linear_acceleration_limit =
    readVectorParams("multirotor.controller.trajectory.linear_acceleration_limit");
  cp.trajectory_controller_params.pid_params.upper_output_saturation = linear_acceleration_limit;
  cp.trajectory_controller_params.pid_params.lower_output_saturation = -linear_acceleration_limit;
  cp.trajectory_controller_params.pid_params.proportional_saturation_flag = true;

  // Controller params Velocity
  cp.velocity_controller_params.pid_params.Kp_gains =
    readVectorParams("multirotor.controller.velocity.kp");
  cp.velocity_controller_params.pid_params.Ki_gains =
    readVectorParams("multirotor.controller.velocity.ki");
  cp.velocity_controller_params.pid_params.Kd_gains =
    readVectorParams("multirotor.controller.velocity.kd");
  cp.velocity_controller_params.pid_params.alpha =
    readVectorParams("multirotor.controller.velocity.alpha");
  cp.velocity_controller_params.pid_params.antiwindup_cte =
    readVectorParams("multirotor.controller.velocity.antiwindup_cte");
  linear_acceleration_limit =
    readVectorParams("multirotor.controller.velocity.linear_acceleration_limit");
  cp.velocity_controller_params.pid_params.upper_output_saturation = linear_acceleration_limit;
  cp.velocity_controller_params.pid_params.lower_output_saturation = -linear_acceleration_limit;
  cp.velocity_controller_params.pid_params.proportional_saturation_flag = true;

  // Controller params Position
  cp.position_controller_params.pid_params.Kp_gains =
    readVectorParams("multirotor.controller.position.kp");
  cp.position_controller_params.pid_params.Ki_gains =
    readVectorParams("multirotor.controller.position.ki");
  cp.position_controller_params.pid_params.Kd_gains =
    readVectorParams("multirotor.controller.position.kd");
  cp.position_controller_params.pid_params.alpha =
    readVectorParams("multirotor.controller.position.alpha");
  cp.position_controller_params.pid_params.antiwindup_cte =
    readVectorParams("multirotor.controller.position.antiwindup_cte");
  Eigen::Vector3d inear_velocity_limit =
    readVectorParams("multirotor.controller.position.linear_velocity_limit");
  cp.position_controller_params.pid_params.upper_output_saturation = inear_velocity_limit;
  cp.position_controller_params.pid_params.lower_output_saturation = -inear_velocity_limit;
  cp.position_controller_params.pid_params.proportional_saturation_flag = true;

  // IMU params
  getParam("multirotor.imu.gyro_noise_var", simulator_params_.imu_params.gyro_noise_var);
  getParam("multirotor.imu.accel_noise_var", simulator_params_.imu_params.accel_noise_var);
  getParam(
    "multirotor.imu.gyro_bias_noise_autocorr_time",
    simulator_params_.imu_params.gyro_bias_noise_autocorr_time);
  getParam(
    "multirotor.imu.accel_bias_noise_autocorr_time",
    simulator_params_.imu_params.accel_bias_noise_autocorr_time);

  // Inertial Odometry params
  getParam("multirotor.inertial_odometry.alpha", simulator_params_.inertial_odometry_params.alpha);
  simulator_params_.inertial_odometry_params.initial_world_orientation = initial_orientation;

  simulator_ = Simulator(simulator_params_);
  simulator_.enable_floor_collision(floor_height);
  RCLCPP_INFO(this->get_logger(), "Parameters read.");
}

void MultirotorSimulatorPlatform::rebaselineTimerBookkeeping()
{
  const rclcpp::Time current_time = this->now();
  last_time_dynamics_ = current_time;
  last_time_control_ = current_time;
  last_time_inertial_odometry_ = current_time;
}

bool MultirotorSimulatorPlatform::isKinematicsFinite(const Kinematics & kinematics)
{
  return kinematics.position.allFinite() &&
         kinematics.orientation.coeffs().allFinite() &&
         kinematics.linear_velocity.allFinite() &&
         kinematics.angular_velocity.allFinite() &&
         kinematics.linear_acceleration.allFinite() &&
         kinematics.angular_acceleration.allFinite();
}

void MultirotorSimulatorPlatform::recoverFromNonFiniteState()
{
  // Restore the dynamics to the last known finite state and reset the
  // components whose internal state may also hold non-finite values.
  simulator_.get_dynamics().set_state(last_finite_state_);
  simulator_.get_controller().reset_controller();
  simulator_.get_imu().reset();
  simulator_.get_inertial_odometry().set_initial_position(
    last_finite_state_.kinematics.position);
  simulator_.get_inertial_odometry().set_initial_orientation(
    last_finite_state_.kinematics.orientation);
  simulator_.get_inertial_odometry().reset();
  control_state_ = last_finite_state_.kinematics;
}

void MultirotorSimulatorPlatform::simulatorTimerCallback()
{
  // Get time
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_dynamics_).seconds();
  last_time_dynamics_ = current_time;

  if (dt <= 0.0 || !std::isfinite(dt)) {
    return;
  }

  // Update simulator. The vendored dynamics model throws
  // std::invalid_argument if dt <= 0 ever reaches it; skip the tick and log
  // instead of letting the exception terminate the node.
  try {
    simulator_.update_dynamics(dt);
    simulator_.update_imu(dt);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skipping dynamics update step (dt=%f): %s", dt, e.what());
  }
}

void MultirotorSimulatorPlatform::simulatorControlTimerCallback()
{
  // Get time
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_control_).seconds();
  last_time_control_ = current_time;

  if (dt <= 0.0 || !std::isfinite(dt)) {
    return;
  }
  control_state_ = simulator_.get_state().kinematics;
  if (using_odom_for_control_) {
    control_state_.position = simulator_.get_odometry().position;
    control_state_.orientation = simulator_.get_odometry().orientation;
  }

  try {
    simulator_.update_controller(dt, control_state_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skipping controller update step (dt=%f): %s", dt, e.what());
  }
}

void MultirotorSimulatorPlatform::simulatorInertialOdometryTimerCallback()
{
  // Get time
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_inertial_odometry_).seconds();
  last_time_inertial_odometry_ = current_time;

  if (dt <= 0.0 || !std::isfinite(dt)) {
    return;
  }

  try {
    simulator_.update_inertial_odometry(dt);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skipping inertial odometry update step (dt=%f): %s", dt, e.what());
  }
}

void MultirotorSimulatorPlatform::simulatorStateTimerCallback()
{
  // Get time
  rclcpp::Time current_time = this->now();

  // Get ground truth simulator state
  const SimulatorState simulator_state = simulator_.get_state();

  // Get odometry simulator state for imu orientation
  const Kinematics odometry_kinematics = simulator_.get_odometry();

  // Get imu simulator
  Eigen::Vector3d imu_angular_velocity, imu_acceleration;
  simulator_.get_imu_measurement(imu_angular_velocity, imu_acceleration);

  // Guard against non-finite state escaping into topics and TF (a NaN in
  // /tf crashes RViz and downstream consumers, and the floor collision
  // clamp cannot catch NaN). Skip this publish cycle and restore the
  // simulator to the last known finite state so it can recover.
  if (!isKinematicsFinite(simulator_state.kinematics) ||
    !isKinematicsFinite(odometry_kinematics) ||
    !imu_angular_velocity.allFinite() || !imu_acceleration.allFinite())
  {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Non-finite simulator state detected, skipping state publish and "
      "restoring last known finite state");
    recoverFromNonFiniteState();
    return;
  }

  // Track the last fully finite state for non-finite state recovery
  last_finite_state_ = simulator_state;

  geometry_msgs::msg::Vector3 imu_angular_velocity_msg;
  imu_angular_velocity_msg.x = imu_angular_velocity.x();
  imu_angular_velocity_msg.y = imu_angular_velocity.y();
  imu_angular_velocity_msg.z = imu_angular_velocity.z();
  geometry_msgs::msg::Vector3 imu_acceleration_msg;
  imu_acceleration_msg.x = imu_acceleration.x();
  imu_acceleration_msg.y = imu_acceleration.y();
  imu_acceleration_msg.z = imu_acceleration.z();

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = current_time;
  imu_msg.header.frame_id = frame_id_baselink_;
  imu_msg.angular_velocity = imu_angular_velocity_msg;
  imu_msg.linear_acceleration = imu_acceleration_msg;
  imu_msg.orientation.w = odometry_kinematics.orientation.w();
  imu_msg.orientation.x = odometry_kinematics.orientation.x();
  imu_msg.orientation.y = odometry_kinematics.orientation.y();
  imu_msg.orientation.z = odometry_kinematics.orientation.z();
  sensor_imu_ptr_->updateData(imu_msg);

  // Get odometry simulator state
  nav_msgs::msg::Odometry odometry;
  as2_interface_.convertToOdom(odometry_kinematics, odometry, current_time);
  sensor_odom_estimate_ptr_->updateData(odometry);

  // Publish ground truth
  geometry_msgs::msg::PoseStamped ground_truth_pose;
  geometry_msgs::msg::TwistStamped ground_truth_twist;
  const Kinematics & kinematics = simulator_state.kinematics;
  as2_interface_.convertToGroundTruth(
    kinematics, ground_truth_pose, ground_truth_twist, current_time);

  sensor_ground_truth_ptr_->updateData(ground_truth_pose, ground_truth_twist);

  // Convert to GPS
  double lat, lon, alt;
  gps_handler_.Local2LatLon(
    ground_truth_pose.pose.position.x, ground_truth_pose.pose.position.y,
    ground_truth_pose.pose.position.z, lat, lon, alt);

  sensor_msgs::msg::NavSatFix gps_msg;
  gps_msg.header.stamp = current_time;
  gps_msg.header.frame_id = frame_id_earth_;
  gps_msg.latitude = lat;
  gps_msg.longitude = lon;
  gps_msg.altitude = alt;
  sensor_gps_ptr_->updateData(gps_msg);

  // Move Gimbal
  gimbal_desired_orientation_.header.stamp = current_time;
  sensor_gimbal_ptr_->updateData(gimbal_desired_orientation_);
}

}  // namespace as2_platform_multirotor_simulator
