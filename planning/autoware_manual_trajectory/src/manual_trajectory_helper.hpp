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

#ifndef MANUAL_TRAJECTORY_HELPER_HPP_
#define MANUAL_TRAJECTORY_HELPER_HPP_

#include "data_structs.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <string>
#include <vector>

namespace autoware::manual_trajectory::helper
{
double calc_forward_length(
  const route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & curr_pose,
  const double default_forward_path_length);

tl::expected<lanelet::ConstLanelets, std::string> get_lanelet_sequence(
  const route_handler::RouteHandler & route_handler, const geometry_msgs::msg::Pose & curr_pose,
  const double backward_sequence_length, const double forward_sequence_length);

double calc_segment_time(
  const autoware_planning_msgs::msg::PathPoint & curr,
  const autoware_planning_msgs::msg::PathPoint & prev);

float calc_steering_angle(const autoware_planning_msgs::msg::PathPoint & p, double wheel_base);

float calc_acc(
  const autoware_planning_msgs::msg::TrajectoryPoint & curr,
  const autoware_planning_msgs::msg::TrajectoryPoint & next);

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> convert_to_trajectory(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path_points,
  double wheel_base);

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> generate_trajectory(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & reference_path,
  const double current_velocity, const double current_acceleration,
  const TrajectoryGenerationParams & params);
std::vector<autoware_planning_msgs::msg::TrajectoryPoint> generate_stop_and_go_sequence(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & reference_path,
  const double current_velocity, const double current_acceleration, double dist_to_stop,
  double stop_duration, const TrajectoryGenerationParams & params);
bool is_near_goal(
  const geometry_msgs::msg::Pose & curr_pose, const geometry_msgs::msg::Pose & goal_pose);
}  // namespace autoware::manual_trajectory::helper

#endif  // MANUAL_TRAJECTORY_HELPER_HPP_
