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

#include "utils.hpp"

#include "fmt/format.h"
#include "str_map.hpp"

#include <autoware/trajectory/utils/closest.hpp>
#include <magic_enum.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
DeparturePoint create_departure_point(
  const Point2d & candidate_point, const DepartureType & departure_type,
  const NodeParam & node_param, const SideKey direction)
{
  DeparturePoint point;
  point.uuid = autoware_utils::to_hex_string(autoware_utils::generate_uuid());
  point.point = candidate_point;
  point.direction = direction;
  point.th_dist_hysteresis = node_param.th_dist_hysteresis_m;
  point.type = departure_type;
  return point;
}

DeparturePoints get_departure_points(
  const ProjectionsToBound & projections_to_bound, const NodeParam & node_param,
  const VehicleInfo & vehicle_info, const double ego_dist_from_traj_front)
{
  DeparturePoints departure_points;
  Side<DeparturePoints> dpts;
  for (const auto direction : g_side_keys) {
    for (const auto & side_to_bound : projections_to_bound[direction]) {
      DeparturePoint point;
      point.uuid = autoware_utils::to_hex_string(autoware_utils::generate_uuid());
      point.lat_dist_to_bound = side_to_bound.lat_dist;
      point.type = side_to_bound.departure_type;
      point.point = side_to_bound.pt_on_bound;
      point.direction = direction;
      point.th_dist_hysteresis = node_param.th_dist_hysteresis_m;
      point.dist_on_traj = side_to_bound.lon_dist_on_ref_traj;
      point.idx_from_ego_traj = side_to_bound.ego_sides_idx;
      point.dist_from_ego =
        point.dist_on_traj - (ego_dist_from_traj_front + vehicle_info.max_longitudinal_offset_m);
      point.can_be_removed = point.dist_from_ego < std::numeric_limits<double>::epsilon() ||
                             point.type == DepartureType::NONE ||
                             point.type == DepartureType::UNKNOWN;

      if (point.can_be_removed) {
        continue;
      }

      dpts[point.direction].push_back(point);
    }

    std::sort(dpts[direction].begin(), dpts[direction].end());
    utils::erase_after_first_match(dpts[direction]);
    std::move(dpts[direction].begin(), dpts[direction].end(), std::back_inserter(departure_points));
  }
  return departure_points;
}

DepartureIntervals init_departure_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const DeparturePoints & departure_points, const VehicleInfo & vehicle_info)
{
  DepartureIntervals departure_intervals;
  size_t idx = 0;
  while (idx < departure_points.size()) {
    DepartureInterval interval;
    interval.start = aw_ref_traj.compute(departure_points[idx].dist_on_traj);
    interval.start_dist_on_traj = departure_points[idx].dist_on_traj;
    interval.candidates.push_back(departure_points[idx]);
    interval.direction = departure_points[idx].direction;

    size_t idx_end = idx + 1;
    while (idx_end < departure_points.size() &&
           departure_points[idx_end].direction == interval.direction &&
           departure_points[idx_end].type == DepartureType::NEAR_BOUNDARY) {
      const auto & prev = departure_points[idx_end - 1];
      const auto & curr = departure_points[idx_end];
      const auto diff = std::abs(curr.dist_on_traj - prev.dist_on_traj);

      if (diff < vehicle_info.max_longitudinal_offset_m && curr.direction == prev.direction) {
        interval.candidates.push_back(curr);
        ++idx_end;
      } else {
        break;
      }
    }

    interval.end = aw_ref_traj.compute(interval.candidates.back().dist_on_traj);
    interval.end_dist_on_traj = interval.candidates.back().dist_on_traj;
    departure_intervals.push_back(interval);
    idx = idx_end;
  }
  return departure_intervals;
}

void update_departure_intervals(
  DepartureIntervals & departure_intervals, DeparturePoints & departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const VehicleInfo vehicle_info,
  const TrajectoryPoint & ref_traj_fr_pt, const double ego_dist_from_traj_front)
{
  for (auto & departure_interval_mut : departure_intervals) {
    // update start end pose
    if (!departure_interval_mut.start_at_traj_front) {
      departure_interval_mut.start_dist_on_traj =
        trajectory::closest(aw_ref_traj, departure_interval_mut.start);
      constexpr auto th_dist_from_start{1.0};
      departure_interval_mut.start_at_traj_front =
        departure_interval_mut.start_dist_on_traj < th_dist_from_start;
    } else {
      departure_interval_mut.start_dist_on_traj = 0.0;
      departure_interval_mut.start = ref_traj_fr_pt;
    }
    departure_interval_mut.end_dist_on_traj =
      trajectory::closest(aw_ref_traj, departure_interval_mut.end);
  }

  // remove if ego already pass the end pose.
  departure_intervals.erase(
    std::remove_if(
      departure_intervals.begin(), departure_intervals.end(),
      [&](const DepartureInterval & interval) {
        if (interval.end_dist_on_traj < ego_dist_from_traj_front) {
          return true;
        }
        auto point_of_curr_traj = aw_ref_traj.compute(interval.end_dist_on_traj);
        return autoware_utils::calc_distance2d(
                 interval.end.pose.position, point_of_curr_traj.pose.position) > 0.25;
      }),
    departure_intervals.end());

  // check if departure point is in between any intervals.
  // if close to end pose, update end pose.
  for (auto & departure_interval_mut : departure_intervals) {
    for (auto & departure_point_mut : departure_points) {
      if (departure_point_mut.can_be_removed) {
        continue;
      }

      if (
        departure_interval_mut.end_dist_on_traj <
          departure_point_mut.dist_on_traj + vehicle_info.max_longitudinal_offset_m &&
        departure_interval_mut.direction == departure_point_mut.direction) {
        // althought name is point on prev traj, we already update them earlier
        departure_interval_mut.end = aw_ref_traj.compute(departure_point_mut.dist_on_traj);
        departure_point_mut.can_be_removed = true;
      }
    }
  }

  if (!departure_intervals.empty()) {
    DepartureIntervals merged;
    merged.push_back(departure_intervals.front());

    for (size_t i = 1; i < departure_intervals.size(); ++i) {
      auto & next_interval_mut = departure_intervals[i];
      auto & curr_interval_mut = merged.back();
      const auto is_same_direction = curr_interval_mut.direction == next_interval_mut.direction;
      if (!is_same_direction) {
        merged.push_back(next_interval_mut);
      }

      const auto is_end_in_between =
        curr_interval_mut.start_dist_on_traj < next_interval_mut.end_dist_on_traj &&
        next_interval_mut.end_dist_on_traj < curr_interval_mut.end_dist_on_traj;
      const auto is_start_in_between =
        curr_interval_mut.start_dist_on_traj < next_interval_mut.start_dist_on_traj &&
        next_interval_mut.start_dist_on_traj < curr_interval_mut.end_dist_on_traj;

      if (is_start_in_between && !is_end_in_between) {
        curr_interval_mut.end = next_interval_mut.end;
        curr_interval_mut.end_dist_on_traj = next_interval_mut.end_dist_on_traj;
        next_interval_mut.has_merged = true;
      } else if (!is_start_in_between && is_end_in_between) {
        curr_interval_mut.start = next_interval_mut.start;
        curr_interval_mut.start_dist_on_traj = next_interval_mut.start_dist_on_traj;
        next_interval_mut.has_merged = true;
      } else if (is_start_in_between && is_end_in_between) {
        next_interval_mut.has_merged = true;
      } else {
        merged.push_back(next_interval_mut);
      }
    }

    departure_intervals = merged;
  }
}

double calc_braking_distance(
  const double abs_velocity, const double max_deceleration, const double delay_time,
  const double dist_error)
{
  return (abs_velocity * abs_velocity) / (2.0 * max_deceleration) + delay_time * abs_velocity +
         dist_error;
}

void erase_after_first_match(DeparturePoints & departure_points)
{
  const auto find_cri_dpt = [](const DeparturePoint & point) {
    return point.type == DepartureType::CRITICAL_DEPARTURE;
  };

  auto crit_dpt_finder =
    std::find_if(departure_points.begin(), departure_points.end(), find_cri_dpt);

  if (
    crit_dpt_finder != departure_points.end() &&
    std::next(crit_dpt_finder) != departure_points.end()) {
    departure_points.erase(std::next(crit_dpt_finder), departure_points.end());
  }
}
}  // namespace autoware::motion_velocity_planner::utils
