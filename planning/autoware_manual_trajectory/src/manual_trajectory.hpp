// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MANUAL_TRAJECTORY_HPP_
#define MANUAL_TRAJECTORY_HPP_

#include "data_structs.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <vector>

namespace autoware::manual_trajectory
{
class ManualTrajectory : public rclcpp::Node
{
public:
  explicit ManualTrajectory(const rclcpp::NodeOptions & options);

private:
  // input
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_planning_msgs::msg::LaneletRoute, autoware_utils_rclcpp::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_map_msgs::msg::LaneletMapBin, autoware_utils_rclcpp::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>
    odometry_subscriber_{this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    geometry_msgs::msg::AccelWithCovarianceStamped>
    acceleration_subscriber_{this, "~/input/accel"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_adapi_v1_msgs::msg::OperationModeState>
    op_mode_state_subscriber_{this, "/api/operation_mode/state"};
  rclcpp::TimerBase::SharedPtr timer_;

  // output
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_traj_{nullptr};

  nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr_{nullptr};
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr_{nullptr};
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr op_mode_state_ptr_{nullptr};
  route_handler::RouteHandler route_handler_;
  vehicle_info_utils::VehicleInfo vehicle_info_;

  TestState state_{TestState::IDLE};
  rclcpp::Time state_start_time_;
  double cached_stop_arc_length_{-1.0};
  TrajectoryGenerationParams traj_gen_param_;

  void take_data();
  std::optional<std::string> has_invalid_data() const;
  void on_timer();
  bool is_autonomous() const;
  void update_test_state(double current_vel, double ego_arc_len, const rclcpp::Time & now);
  tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
  generate_centerline_path();

  template <typename... Args>
  void info(const char * fmt, Args... args)
  {
    RCLCPP_INFO(this->get_logger(), fmt, args...);
  }

  template <typename... Args>
  void warn_throttle(const char * fmt, Args... args)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, fmt, args...);
  }

  void warn_throttle(const char * fmt)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s", fmt);
  }
};
}  // namespace autoware::manual_trajectory

#endif  // MANUAL_TRAJECTORY_HPP_
