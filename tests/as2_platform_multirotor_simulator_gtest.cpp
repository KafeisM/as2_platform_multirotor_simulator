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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
 * @file as2_platform_multirotor_simulator_gtest.cpp
 *
 * MultirotorSimulatorPlatform gtets
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "as2_platform_multirotor_simulator/as2_platform_multirotor_simulator.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace as2_platform_multirotor_simulator
{

using ResetSimulatorState = as2_platform_multirotor_simulator::srv::ResetSimulatorState;

constexpr double kPostResetLinearSpeedTolerance = 0.05;
constexpr double kPostResetAngularSpeedTolerance = 0.1;

class ResetTestMultirotorSimulatorPlatform : public MultirotorSimulatorPlatform
{
public:
  explicit ResetTestMultirotorSimulatorPlatform(const rclcpp::NodeOptions & options)
  : MultirotorSimulatorPlatform(options) {}

  bool applyControlMode(const as2_msgs::msg::ControlMode & control_mode)
  {
    return setPlatformControlMode(control_mode);
  }

  bool sendVelocityReference(const Eigen::Vector3d & velocity, const double yaw_rate)
  {
    command_twist_msg_.header.frame_id = "earth";
    command_twist_msg_.header.stamp = this->now();
    command_twist_msg_.twist.linear.x = velocity.x();
    command_twist_msg_.twist.linear.y = velocity.y();
    command_twist_msg_.twist.linear.z = velocity.z();
    command_twist_msg_.twist.angular.z = yaw_rate;
    return ownSendCommand();
  }

  bool sendPositionReference(const Eigen::Vector3d & position, const double yaw)
  {
    command_pose_msg_.header.frame_id = "earth";
    command_pose_msg_.header.stamp = this->now();
    command_pose_msg_.pose.position.x = position.x();
    command_pose_msg_.pose.position.y = position.y();
    command_pose_msg_.pose.position.z = position.z();
    as2::frame::eulerToQuaternion(0.0, 0.0, yaw, command_pose_msg_.pose.orientation);

    command_twist_msg_.header.frame_id = "earth";
    command_twist_msg_.header.stamp = this->now();
    command_twist_msg_.twist.linear.x = 1.0;
    command_twist_msg_.twist.linear.y = 1.0;
    command_twist_msg_.twist.linear.z = 1.0;
    return ownSendCommand();
  }

  bool sendTrajectoryReference(
    const Eigen::Vector3d & position, const Eigen::Vector3d & velocity,
    const Eigen::Vector3d & acceleration, const double yaw)
  {
    command_trajectory_msg_.header.frame_id = "earth";
    command_trajectory_msg_.header.stamp = this->now();

    as2_msgs::msg::TrajectoryPoint setpoint;
    setpoint.position.x = position.x();
    setpoint.position.y = position.y();
    setpoint.position.z = position.z();
    setpoint.twist.x = velocity.x();
    setpoint.twist.y = velocity.y();
    setpoint.twist.z = velocity.z();
    setpoint.acceleration.x = acceleration.x();
    setpoint.acceleration.y = acceleration.y();
    setpoint.acceleration.z = acceleration.z();
    setpoint.yaw_angle = yaw;
    command_trajectory_msg_.setpoints.clear();
    command_trajectory_msg_.setpoints.push_back(setpoint);
    return ownSendCommand();
  }
};

struct GroundTruthCapture
{
  std::optional<geometry_msgs::msg::PoseStamped> pose;
  std::optional<geometry_msgs::msg::TwistStamped> twist;
};

rclcpp::NodeOptions makeNodeOptions(const std::string & name_space)
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_multirotor_simulator");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  const std::string platform_config_file = package_path + "/config/platform_config_file.yaml";
  const std::string world_config = package_path + "/config/world_config.yaml";
  const std::string uav_config = package_path + "/config/uav_config.yaml";

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=" + std::string("/") + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "control_modes_file:=" + control_modes_config_file,
    "--params-file",
    platform_config_file,
    "--params-file",
    world_config,
    "--params-file",
    uav_config,
  };

  rclcpp::NodeOptions node_options;
  node_options.arguments(node_args);
  return node_options;
}

std::shared_ptr<MultirotorSimulatorPlatform> get_node(
  const std::string & name_space = "multirotor_simulator")
{
  return std::make_shared<MultirotorSimulatorPlatform>(makeNodeOptions(name_space));
}

std::shared_ptr<ResetTestMultirotorSimulatorPlatform> get_reset_test_node(
  const std::string & name_space)
{
  return std::make_shared<ResetTestMultirotorSimulatorPlatform>(makeNodeOptions(name_space));
}

ResetSimulatorState::Request makeResetRequest(
  const double x, const double y, const double z, const double yaw)
{
  ResetSimulatorState::Request request;
  request.x = x;
  request.y = y;
  request.z = z;
  request.yaw = yaw;
  request.position_tolerance = 0.05;
  request.yaw_tolerance = 0.05;
  request.linear_speed_tolerance = 0.01;
  request.angular_speed_tolerance = 0.01;
  return request;
}

std::shared_ptr<ResetSimulatorState::Response> callResetService(
  const rclcpp::Client<ResetSimulatorState>::SharedPtr & client,
  rclcpp::executors::SingleThreadedExecutor & executor,
  const ResetSimulatorState::Request & request)
{
  auto future = client->async_send_request(std::make_shared<ResetSimulatorState::Request>(request));
  EXPECT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  return future.get();
}

void waitForResetService(
  const rclcpp::Client<ResetSimulatorState>::SharedPtr & client,
  rclcpp::executors::SingleThreadedExecutor & executor)
{
  const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!client->wait_for_service(std::chrono::milliseconds(50)) &&
    std::chrono::steady_clock::now() < wait_deadline)
  {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(client->service_is_ready());
}

void spinExecutorFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
}

void expectGroundTruthMatchesReset(
  const GroundTruthCapture & ground_truth, const ResetSimulatorState::Request & request)
{
  ASSERT_TRUE(ground_truth.pose.has_value());
  ASSERT_TRUE(ground_truth.twist.has_value());

  const auto & pose = ground_truth.pose.value().pose;
  const auto & twist = ground_truth.twist.value().twist;
  const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Vector3d target(request.x, request.y, request.z);
  const double current_yaw = as2::frame::getYawFromQuaternion(pose.orientation);
  const double yaw_error = std::abs(
    std::atan2(std::sin(current_yaw - request.yaw), std::cos(current_yaw - request.yaw)));
  const Eigen::Vector3d linear_velocity(twist.linear.x, twist.linear.y, twist.linear.z);
  const Eigen::Vector3d angular_velocity(twist.angular.x, twist.angular.y, twist.angular.z);

  EXPECT_LE((position - target).norm(), request.position_tolerance);
  EXPECT_LE(yaw_error, request.yaw_tolerance);
  EXPECT_LE(linear_velocity.norm(), kPostResetLinearSpeedTolerance);
  EXPECT_LE(angular_velocity.norm(), kPostResetAngularSpeedTolerance);
}

void expectResetArtifactsCleared(const ResetSimulatorState::Response & response)
{
  EXPECT_LE(response.linear_acceleration_norm, 1e-9);
  EXPECT_LE(response.angular_acceleration_norm, 1e-9);
  EXPECT_LE(response.force_norm, 1e-9);
  EXPECT_LE(response.torque_norm, 1e-9);
  EXPECT_LE(response.motor_angular_velocity_norm, 1e-9);
  EXPECT_LE(response.motor_angular_acceleration_norm, 1e-9);
  EXPECT_TRUE(response.dynamics_reset_applied);
  EXPECT_TRUE(response.motor_state_reset_applied);
  EXPECT_TRUE(response.controller_reset_applied);
  EXPECT_TRUE(response.imu_reset_applied);
  EXPECT_TRUE(response.odometry_reset_applied);
  EXPECT_TRUE(response.references_reset_applied);
  EXPECT_TRUE(response.dynamic_artifacts_cleared);
}

TEST(MultirotorSimulatorPlatform, test_constructor)
{
  EXPECT_NO_THROW(std::shared_ptr<MultirotorSimulatorPlatform> node = get_node("test_constructor"));
}

TEST(MultirotorSimulatorPlatform, test_virtual_methods)
{
  std::shared_ptr<MultirotorSimulatorPlatform> node = get_node("test_virtual_methods");
  EXPECT_NO_THROW(node->configureSensors());
  EXPECT_NO_THROW(node->ownSetArmingState(true));
  EXPECT_NO_THROW(node->ownSetOffboardControl(true));
  as2_msgs::msg::ControlMode msg;
  msg.control_mode = as2_msgs::msg::ControlMode::ACRO;
  EXPECT_NO_THROW(node->ownSetPlatformControlMode(msg));
  EXPECT_NO_THROW(node->ownSendCommand());
  EXPECT_NO_THROW(node->ownStopPlatform());
  EXPECT_NO_THROW(node->ownKillSwitch());
  // EXPECT_NO_THROW(node->ownTakeoff()); // Block without an Executor
  // EXPECT_NO_THROW(node->ownLand()); // Block without an Executor

  // Spin node
  rclcpp::spin_some(node);
}

TEST(MultirotorSimulatorPlatform, reset_service_is_discoverable_and_callable)
{
  const std::string server_namespace = "test_reset_service";
  auto server_node = get_node(server_namespace);
  auto client_node = std::make_shared<rclcpp::Node>("reset_service_client");
  auto client = client_node->create_client<ResetSimulatorState>(
    "/" + server_namespace + "/platform/reset_simulator_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  waitForResetService(client, executor);

  auto valid_request = makeResetRequest(4.0, -2.0, 1.0, 0.5);
  auto valid_response = callResetService(client, executor, valid_request);
  EXPECT_TRUE(valid_response->success);
  EXPECT_EQ(valid_response->message, "reset state applied");
  EXPECT_TRUE(std::isfinite(valid_response->position_error));
  EXPECT_TRUE(std::isfinite(valid_response->yaw_error));
  EXPECT_TRUE(std::isfinite(valid_response->linear_speed_norm));
  EXPECT_TRUE(std::isfinite(valid_response->angular_speed_norm));
  EXPECT_TRUE(std::isfinite(valid_response->linear_acceleration_norm));
  EXPECT_TRUE(std::isfinite(valid_response->angular_acceleration_norm));
  EXPECT_TRUE(std::isfinite(valid_response->force_norm));
  EXPECT_TRUE(std::isfinite(valid_response->torque_norm));
  EXPECT_TRUE(std::isfinite(valid_response->motor_angular_velocity_norm));
  EXPECT_TRUE(std::isfinite(valid_response->motor_angular_acceleration_norm));
  EXPECT_LE(valid_response->position_error, valid_request.position_tolerance);
  EXPECT_LE(valid_response->yaw_error, valid_request.yaw_tolerance);
  EXPECT_LE(valid_response->linear_speed_norm, valid_request.linear_speed_tolerance);
  EXPECT_LE(valid_response->angular_speed_norm, valid_request.angular_speed_tolerance);
  expectResetArtifactsCleared(*valid_response);

  auto second_valid_response = callResetService(client, executor, valid_request);
  EXPECT_TRUE(second_valid_response->success);
  EXPECT_LE(second_valid_response->position_error, valid_request.position_tolerance);
  EXPECT_LE(second_valid_response->yaw_error, valid_request.yaw_tolerance);
  expectResetArtifactsCleared(*second_valid_response);

  auto invalid_request = valid_request;
  invalid_request.x = std::numeric_limits<double>::quiet_NaN();
  auto invalid_response = callResetService(client, executor, invalid_request);
  EXPECT_FALSE(invalid_response->success);
  EXPECT_EQ(invalid_response->message, "reset target pose/yaw must contain only finite values");
  EXPECT_TRUE(std::isfinite(invalid_response->position_error));
  EXPECT_TRUE(std::isfinite(invalid_response->yaw_error));
  EXPECT_TRUE(std::isfinite(invalid_response->linear_speed_norm));
  EXPECT_TRUE(std::isfinite(invalid_response->angular_speed_norm));

  executor.remove_node(client_node);
  executor.remove_node(server_node);
}

TEST(MultirotorSimulatorPlatform, reset_service_rejects_non_finite_tolerances)
{
  const std::string server_namespace = "test_reset_tolerances";
  auto server_node = get_node(server_namespace);
  auto client_node = std::make_shared<rclcpp::Node>("reset_tolerance_client");
  auto client = client_node->create_client<ResetSimulatorState>(
    "/" + server_namespace + "/platform/reset_simulator_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  waitForResetService(client, executor);

  auto invalid_position_tolerance_request = makeResetRequest(1.0, 2.0, 1.5, -0.25);
  invalid_position_tolerance_request.position_tolerance =
    std::numeric_limits<double>::quiet_NaN();
  auto invalid_position_tolerance_response = callResetService(
    client, executor, invalid_position_tolerance_request);
  EXPECT_FALSE(invalid_position_tolerance_response->success);
  EXPECT_EQ(
    invalid_position_tolerance_response->message,
    "reset tolerances must contain only finite values");
  EXPECT_TRUE(std::isfinite(invalid_position_tolerance_response->position_error));
  EXPECT_TRUE(std::isfinite(invalid_position_tolerance_response->yaw_error));
  EXPECT_TRUE(std::isfinite(invalid_position_tolerance_response->linear_speed_norm));
  EXPECT_TRUE(std::isfinite(invalid_position_tolerance_response->angular_speed_norm));

  auto invalid_yaw_tolerance_request = makeResetRequest(1.0, 2.0, 1.5, -0.25);
  invalid_yaw_tolerance_request.yaw_tolerance = std::numeric_limits<double>::quiet_NaN();
  auto invalid_yaw_tolerance_response = callResetService(
    client, executor, invalid_yaw_tolerance_request);
  EXPECT_FALSE(invalid_yaw_tolerance_response->success);
  EXPECT_EQ(
    invalid_yaw_tolerance_response->message,
    "reset tolerances must contain only finite values");

  auto invalid_linear_speed_tolerance_request = makeResetRequest(1.0, 2.0, 1.5, -0.25);
  invalid_linear_speed_tolerance_request.linear_speed_tolerance =
    std::numeric_limits<double>::quiet_NaN();
  auto invalid_linear_speed_tolerance_response = callResetService(
    client, executor, invalid_linear_speed_tolerance_request);
  EXPECT_FALSE(invalid_linear_speed_tolerance_response->success);
  EXPECT_EQ(
    invalid_linear_speed_tolerance_response->message,
    "reset tolerances must contain only finite values");

  auto invalid_angular_speed_tolerance_request = makeResetRequest(1.0, 2.0, 1.5, -0.25);
  invalid_angular_speed_tolerance_request.angular_speed_tolerance =
    std::numeric_limits<double>::infinity();
  auto invalid_angular_speed_tolerance_response = callResetService(
    client, executor, invalid_angular_speed_tolerance_request);
  EXPECT_FALSE(invalid_angular_speed_tolerance_response->success);
  EXPECT_EQ(
    invalid_angular_speed_tolerance_response->message,
    "reset tolerances must contain only finite values");

  executor.remove_node(client_node);
  executor.remove_node(server_node);
}

TEST(MultirotorSimulatorPlatform, reset_service_rejects_non_positive_tolerances)
{
  const std::string server_namespace = "test_reset_non_positive_tolerances";
  auto server_node = get_node(server_namespace);
  auto client_node = std::make_shared<rclcpp::Node>("reset_non_positive_tolerance_client");
  auto client = client_node->create_client<ResetSimulatorState>(
    "/" + server_namespace + "/platform/reset_simulator_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  waitForResetService(client, executor);

  auto zero_tolerance_request = makeResetRequest(1.0, 2.0, 1.5, -0.25);
  zero_tolerance_request.position_tolerance = 0.0;
  auto zero_tolerance_response = callResetService(client, executor, zero_tolerance_request);
  EXPECT_FALSE(zero_tolerance_response->success);
  EXPECT_EQ(zero_tolerance_response->message, "reset tolerances must be strictly positive");
  EXPECT_TRUE(std::isfinite(zero_tolerance_response->position_error));
  EXPECT_TRUE(std::isfinite(zero_tolerance_response->yaw_error));
  EXPECT_TRUE(std::isfinite(zero_tolerance_response->linear_speed_norm));
  EXPECT_TRUE(std::isfinite(zero_tolerance_response->angular_speed_norm));

  auto negative_tolerance_request = makeResetRequest(1.0, 2.0, 1.5, -0.25);
  negative_tolerance_request.angular_speed_tolerance = -0.01;
  auto negative_tolerance_response = callResetService(
    client, executor, negative_tolerance_request);
  EXPECT_FALSE(negative_tolerance_response->success);
  EXPECT_EQ(
    negative_tolerance_response->message,
    "reset tolerances must be strictly positive");

  executor.remove_node(client_node);
  executor.remove_node(server_node);
}

TEST(MultirotorSimulatorPlatform, reset_service_clears_stale_motion_state)
{
  const std::string server_namespace = "test_reset_stale_motion";
  auto server_node = get_reset_test_node(server_namespace);
  auto client_node = std::make_shared<rclcpp::Node>("reset_stale_motion_client");
  auto client = client_node->create_client<ResetSimulatorState>(
    "/" + server_namespace + "/platform/reset_simulator_state");
  GroundTruthCapture ground_truth;
  auto pose_subscription = client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/" + server_namespace + "/ground_truth/pose",
    as2_names::topics::ground_truth::qos,
    [&ground_truth](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      ground_truth.pose = *msg;
    });
  auto twist_subscription = client_node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/" + server_namespace + "/ground_truth/twist",
    as2_names::topics::ground_truth::qos,
    [&ground_truth](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      ground_truth.twist = *msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  waitForResetService(client, executor);

  ASSERT_TRUE(server_node->ownSetArmingState(true));
  as2_msgs::msg::ControlMode velocity_control_mode;
  velocity_control_mode.control_mode = as2_msgs::msg::ControlMode::SPEED;
  velocity_control_mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  velocity_control_mode.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  ASSERT_TRUE(server_node->applyControlMode(velocity_control_mode));
  ASSERT_TRUE(server_node->sendVelocityReference(Eigen::Vector3d(6.0, -3.0, 0.0), 1.5));
  spinExecutorFor(executor, std::chrono::milliseconds(100));

  as2_msgs::msg::ControlMode position_control_mode;
  position_control_mode.control_mode = as2_msgs::msg::ControlMode::POSITION;
  position_control_mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  position_control_mode.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  ASSERT_TRUE(server_node->applyControlMode(position_control_mode));
  ASSERT_TRUE(server_node->sendPositionReference(Eigen::Vector3d(-4.0, 3.0, 2.5), -1.0));
  spinExecutorFor(executor, std::chrono::milliseconds(100));

  as2_msgs::msg::ControlMode trajectory_control_mode;
  trajectory_control_mode.control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;
  trajectory_control_mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  trajectory_control_mode.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  ASSERT_TRUE(server_node->applyControlMode(trajectory_control_mode));
  ASSERT_TRUE(
    server_node->sendTrajectoryReference(
      Eigen::Vector3d(5.0, -5.0, 3.0), Eigen::Vector3d(4.0, 0.0, 0.0),
      Eigen::Vector3d::Zero(), 2.0));
  spinExecutorFor(executor, std::chrono::milliseconds(100));

  const auto first_request = makeResetRequest(2.0, -1.5, 1.25, 0.75);
  auto first_response = callResetService(client, executor, first_request);
  ASSERT_TRUE(first_response->success);
  EXPECT_LE(first_response->position_error, first_request.position_tolerance);
  EXPECT_LE(first_response->yaw_error, first_request.yaw_tolerance);
  EXPECT_LE(first_response->linear_speed_norm, first_request.linear_speed_tolerance);
  EXPECT_LE(first_response->angular_speed_norm, first_request.angular_speed_tolerance);
  expectResetArtifactsCleared(*first_response);

  ground_truth.pose.reset();
  ground_truth.twist.reset();
  spinExecutorFor(executor, std::chrono::milliseconds(300));
  expectGroundTruthMatchesReset(ground_truth, first_request);

  const auto second_request = makeResetRequest(-1.0, 0.5, 1.75, -1.25);
  auto second_response = callResetService(client, executor, second_request);
  EXPECT_TRUE(second_response->success);
  EXPECT_LE(second_response->position_error, second_request.position_tolerance);
  EXPECT_LE(second_response->yaw_error, second_request.yaw_tolerance);
  EXPECT_LE(second_response->linear_speed_norm, second_request.linear_speed_tolerance);
  EXPECT_LE(second_response->angular_speed_norm, second_request.angular_speed_tolerance);
  expectResetArtifactsCleared(*second_response);

  ground_truth.pose.reset();
  ground_truth.twist.reset();
  spinExecutorFor(executor, std::chrono::milliseconds(300));
  expectGroundTruthMatchesReset(ground_truth, second_request);

  executor.remove_node(client_node);
  executor.remove_node(server_node);
}

TEST(MultirotorSimulatorPlatform, reset_service_holds_requested_altitude_from_disarmed_state)
{
  const std::string server_namespace = "test_reset_disarmed_altitude_hold";
  auto server_node = get_reset_test_node(server_namespace);
  auto client_node = std::make_shared<rclcpp::Node>("reset_disarmed_altitude_hold_client");
  auto client = client_node->create_client<ResetSimulatorState>(
    "/" + server_namespace + "/platform/reset_simulator_state");
  GroundTruthCapture ground_truth;
  auto pose_subscription = client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/" + server_namespace + "/ground_truth/pose",
    as2_names::topics::ground_truth::qos,
    [&ground_truth](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      ground_truth.pose = *msg;
    });
  auto twist_subscription = client_node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/" + server_namespace + "/ground_truth/twist",
    as2_names::topics::ground_truth::qos,
    [&ground_truth](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      ground_truth.twist = *msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  waitForResetService(client, executor);

  const auto hover_request = makeResetRequest(0.0, 0.0, 1.0, 0.0);
  auto response = callResetService(client, executor, hover_request);
  ASSERT_TRUE(response->success);
  EXPECT_LE(response->position_error, hover_request.position_tolerance);
  EXPECT_LE(response->yaw_error, hover_request.yaw_tolerance);
  EXPECT_LE(response->linear_speed_norm, hover_request.linear_speed_tolerance);
  EXPECT_LE(response->angular_speed_norm, hover_request.angular_speed_tolerance);
  expectResetArtifactsCleared(*response);

  ground_truth.pose.reset();
  ground_truth.twist.reset();
  spinExecutorFor(executor, std::chrono::milliseconds(600));
  expectGroundTruthMatchesReset(ground_truth, hover_request);

  ASSERT_TRUE(server_node->ownSetArmingState(false));
  const auto offset_hover_request = makeResetRequest(-0.5, 0.75, 1.5, 0.6);
  auto offset_response = callResetService(client, executor, offset_hover_request);
  ASSERT_TRUE(offset_response->success);
  EXPECT_LE(offset_response->position_error, offset_hover_request.position_tolerance);
  EXPECT_LE(offset_response->yaw_error, offset_hover_request.yaw_tolerance);
  EXPECT_LE(offset_response->linear_speed_norm, offset_hover_request.linear_speed_tolerance);
  EXPECT_LE(offset_response->angular_speed_norm, offset_hover_request.angular_speed_tolerance);
  expectResetArtifactsCleared(*offset_response);

  ground_truth.pose.reset();
  ground_truth.twist.reset();
  spinExecutorFor(executor, std::chrono::milliseconds(600));
  expectGroundTruthMatchesReset(ground_truth, offset_hover_request);

  executor.remove_node(client_node);
  executor.remove_node(server_node);
}

}  // namespace as2_platform_multirotor_simulator

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
