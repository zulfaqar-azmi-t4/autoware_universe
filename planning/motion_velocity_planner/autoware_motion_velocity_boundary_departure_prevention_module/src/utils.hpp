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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "parameters.hpp"

#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{

template <typename Container, typename Predicate>
void erase_if(Container & container, Predicate pred)
{
  for (auto it = container.begin(); it != container.end();) {
    if (pred(*it)) {  // For map, *it is a std::pair, so pred should handle pair<Key, Value>
      it = container.erase(it);
    } else {
      ++it;
    }
  }
}

inline geometry_msgs::msg::Point to_geom_pt(const Point2d & point)
{
  return autoware_utils::to_msg(point.to_3d(0.0));
}

DeparturePoint create_departure_point(
  const Point2d & candidate_point, const DepartureType & departure_type,
  const NodeParam & node_param, std::string_view direction);

DeparturePoints get_departure_points(
  const ClosestProjectionsToBound & projections_to_bound, const NodeParam & node_param,
  const VehicleInfo & vehicle_info, const double ego_dist_from_traj_front);
DepartureIntervals init_departure_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const DeparturePoints & departure_points, const VehicleInfo & vehicle_info);

void update_departure_intervals(
  DepartureIntervals & departure_intervals, DeparturePoints & departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const VehicleInfo vehicle_info,
  const TrajectoryPoint & ref_traj_fr_pt, const double ego_dist_from_traj_front);

double calc_braking_distance(
  const double abs_velocity, const double max_deceleration, const double delay_time,
  const double dist_error);

void erase_after_first_match(DeparturePoints & departure_points);
}  // namespace autoware::motion_velocity_planner::utils
#endif  // UTILS_HPP_
