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
  const NodeParam & node_param, std::string_view direction)
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
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const AbnormalityType<DepartureStatuses> & departure_types_idx,
  const AbnormalityType<SideToBoundPojections> & side_to_bound_projections,
  const NodeParam & node_param, const VehicleInfo & vehicle_info,
  const double ego_dist_from_traj_front)
{
  DeparturePoints departure_points;
  for (const auto abnormality_key : abnormality_keys) {
    Side<DeparturePoints> dpts;
    for (const auto direction : side_keys) {
      auto & departure_statuses = departure_types_idx[abnormality_key][direction];
      const auto & side_to_bound = side_to_bound_projections[abnormality_key][direction];
      for (const auto & [status, idx] : departure_statuses) {
        const auto & curr_side = side_to_bound[idx];

        DeparturePoint point;
        point.uuid = autoware_utils::to_hex_string(autoware_utils::generate_uuid());
        point.lat_dist_to_bound = curr_side.lat_dist;
        point.type = status;
        point.point = curr_side.pt_on_bound;
        point.direction = direction;
        point.th_dist_hysteresis = node_param.th_dist_hysteresis_m;
        point.th_lat_dist_to_bounday_hyteresis = std::invoke([&]() -> double {
          if (point.type == DepartureType::CRITICAL_DEPARTURE) {
            return node_param.stop_before_departure.th_dist_to_boundary_m[direction];
          }
          if (point.type == DepartureType::APPROACHING_DEPARTURE) {
            return node_param.slow_down_before_departure.th_dist_to_boundary_m[direction];
          }

          if (point.type == DepartureType::NEAR_BOUNDARY) {
            return node_param.slow_down_near_boundary.th_dist_to_boundary_m[direction];
          }

          return std::numeric_limits<double>::max();
        });

        point.dist_on_traj = trajectory::closest(aw_ref_traj, point.to_geom_pt(0.0));
        point.dist_from_ego =
          point.dist_on_traj - (ego_dist_from_traj_front + vehicle_info.max_longitudinal_offset_m);
        point.can_be_removed = point.dist_from_ego < std::numeric_limits<double>::epsilon();

        if (point.can_be_removed) {
          continue;
        }

        point.point_on_prev_traj = aw_ref_traj.compute(point.dist_on_traj);
        dpts[point.direction].push_back(point);
      }
      std::sort(dpts[direction].begin(), dpts[direction].end());
      utils::erase_after_first_match(dpts[direction]);
      std::move(
        dpts[direction].begin(), dpts[direction].end(), std::back_inserter(departure_points));
    }
  }
  return departure_points;
}

DepartureIntervals init_departure_intervals(
  const DeparturePoints & departure_points, const VehicleInfo & vehicle_info)
{
  DepartureIntervals departure_intervals;
  size_t idx = 0;
  while (idx < departure_points.size()) {
    DepartureInterval interval;
    interval.start = departure_points[idx].point_on_prev_traj;
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

    interval.end = interval.candidates.back().point_on_prev_traj;
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
        departure_interval_mut.end = departure_point_mut.point_on_prev_traj;
        departure_point_mut.can_be_removed = true;
      }
    }
  }
}

AbnormalityType<DepartureStatuses> check_departure_status(
  const AbnormalityType<SideToBoundPojections> & side_to_bound_projections, const NodeParam & param,
  [[maybe_unused]] const double curr_vel)
{
  AbnormalityType<DepartureStatuses> stats;
  const auto assign_status = [](
                               const double lat_dist_m, const auto & param,
                               const auto & abnormality_key,
                               const auto & side_key) -> DepartureType {
    const auto stop_before_dpt = param.stop_before_departure.th_dist_to_boundary_m[side_key];
    const auto slow_before_dpt = param.slow_down_before_departure.th_dist_to_boundary_m[side_key];
    const auto slow_near_bound = param.slow_down_near_boundary.th_dist_to_boundary_m[side_key];

    if (std::abs(lat_dist_m) < stop_before_dpt && abnormality_key == "steering") {
      return DepartureType::CRITICAL_DEPARTURE;
    }

    if (abnormality_key == "steering") {
      return DepartureType::NONE;
    }

    if (std::abs(lat_dist_m) < slow_before_dpt) {
      return DepartureType::APPROACHING_DEPARTURE;
    }

    if (std::abs(lat_dist_m) < slow_near_bound) {
      return DepartureType::NEAR_BOUNDARY;
    }

    return DepartureType::NONE;
  };

  for (const auto abnormality_key : abnormality_keys) {
    for (const auto side_key : side_keys) {
      for (const auto & [pt_on_ego, pg_on_bound, segment, lat_dist_m, idx_from_orig] :
           side_to_bound_projections[abnormality_key][side_key]) {
        const auto status = assign_status(lat_dist_m, param, abnormality_key, side_key);
        if (status != DepartureType::NONE && status != DepartureType::UNKNOWN) {
          stats[abnormality_key][side_key].emplace_back(status, idx_from_orig);
        }
      }
    }
  }

  return stats;
}

double calc_braking_distance(
  const double abs_velocity, const double max_deceleration, const double delay_time,
  const double dist_error)
{
  return (abs_velocity * abs_velocity) / (2.0 * max_deceleration) + delay_time * abs_velocity +
         dist_error;
}

std::vector<std::pair<size_t, size_t>> get_traj_indices_candidates(
  const std::vector<DepartureTypeIdx> & departure_stats, const EgoSides & ego_sides,
  const double ego_length)
{
  if (departure_stats.empty()) {
    return {};
  }
  std::vector<std::pair<size_t, size_t>> slow_down_candidate_idx;
  size_t p1 = 0;
  size_t p2 = 0;
  for (; p2 + 1 < departure_stats.size(); ++p2) {
    const auto [stat1, idx1] = departure_stats.at(p2);
    const auto [stat2, idx2] = departure_stats.at(p2 + 1);
    if (idx1 >= ego_sides.size() || idx2 >= ego_sides.size()) {
      break;
    }
    const auto diff =
      std::abs(ego_sides.at(idx2).dist_from_start - ego_sides.at(idx1).dist_from_start);
    // have to compare between two arrays
    const auto cond1 = diff < ego_length;
    const auto cond2 = p2 < departure_stats.size() - 1;
    if (cond1 && cond2) {
      ++p2;
      continue;
    }
    slow_down_candidate_idx.emplace_back(
      departure_stats.at(p1).second, departure_stats.at(p2).second);
    p1 = p2;
  }

  if (p1 <= p2 && p2 != 0UL) {
    slow_down_candidate_idx.emplace_back(
      departure_stats.at(p1).second, departure_stats.at(p2 - 1).second);
  }
  return slow_down_candidate_idx;
}

void erase_after_first_match(DeparturePoints & departure_points)
{
  const auto find_cri_dpt = [](const DeparturePoint & point) {
    return point.type == DepartureType::CRITICAL_DEPARTURE ||
           point.type == DepartureType::APPROACHING_DEPARTURE;
  };

  auto crit_dpt_finder =
    std::find_if(departure_points.begin(), departure_points.end(), find_cri_dpt);

  if (
    crit_dpt_finder != departure_points.end() &&
    std::next(crit_dpt_finder) != departure_points.end()) {
    departure_points.erase(std::next(crit_dpt_finder), departure_points.end());
  }
}
double compute_braking_distance(
  double v_init, double v_end, double a, double j, double t_braking_delay)
{
  // Phase 1: jerk phase
  const double t1 = a / j;
  const double d1 = v_init * t1 - (a / 6.0) * t1 * t1;

  // Midpoint velocity after jerk phase
  const double v_mid = v_init - (a / 2.0) * t1;

  // Phase 2: constant deceleration
  const double dv2 = v_mid - v_end;
  const double t2 = dv2 / a;
  const double d2 = ((v_mid + v_end) / 2.0) * t2;

  return d1 + d2 + (5.0 / 3.6) * t_braking_delay;
}

}  // namespace autoware::motion_velocity_planner::utils
