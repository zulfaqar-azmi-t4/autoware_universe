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

#include <autoware/trajectory/utils/closest.hpp>
#include <magic_enum.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view.hpp>
#include <tf2/convert.hpp>

#include <algorithm>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
DepartureIntervals init_departure_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const DeparturePoints & departure_points, const double vehicle_length_m, const SideKey side_key)
{
  DepartureIntervals departure_intervals;
  size_t idx = 0;
  while (idx < departure_points.size()) {
    DepartureInterval interval;
    interval.start = aw_ref_traj.compute(departure_points[idx].dist_on_traj);
    interval.start_dist_on_traj = departure_points[idx].dist_on_traj;
    interval.candidates.push_back(departure_points[idx]);
    interval.side_key = side_key;

    size_t idx_end = idx + 1;

    while (idx_end < departure_points.size()) {
      const auto & curr = departure_points[idx_end];
      if (
        curr.departure_type != DepartureType::NEAR_BOUNDARY &&
        curr.departure_type != DepartureType::APPROACHING_DEPARTURE) {
        break;
      }

      if (curr.can_be_removed) {
        continue;
      }

      const auto & prev = departure_points[idx_end - 1];
      const auto diff = std::abs(curr.dist_on_traj - prev.dist_on_traj);

      if (diff < vehicle_length_m) {
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

DepartureIntervals init_departure_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const Side<DeparturePoints> & departure_points, const double vehicle_length)
{
  DepartureIntervals departure_intervals;
  for (const auto side_key : g_side_keys) {
    auto dpt_pts =
      init_departure_intervals(aw_ref_traj, departure_points[side_key], vehicle_length, side_key);
    std::move(dpt_pts.begin(), dpt_pts.end(), std::back_inserter(departure_intervals));
  }
  return departure_intervals;
}

void update_departure_intervals_poses(
  DepartureIntervals & departure_intervals,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const TrajectoryPoint & ref_traj_fr_pt, const double ego_dist_from_traj_front,
  const double th_pt_shift_dist_m, const double th_pt_shift_angle_rad)
{
  for (auto & dpt_pt : departure_intervals) {
    // update start end pose
    if (!dpt_pt.start_at_traj_front) {
      dpt_pt.start_dist_on_traj = trajectory::closest(aw_ref_traj, dpt_pt.start);
      constexpr auto th_dist_from_start{1.0};
      dpt_pt.start_at_traj_front = dpt_pt.start_dist_on_traj < th_dist_from_start;
    }

    if (dpt_pt.start_at_traj_front) {
      dpt_pt.start_dist_on_traj = 0.0;
      dpt_pt.start = ref_traj_fr_pt;
    }

    dpt_pt.end_dist_on_traj = trajectory::closest(aw_ref_traj, dpt_pt.end);
  }

  // remove if ego already pass the end pose.
  utils::remove_if(departure_intervals, [&](const DepartureInterval & interval) {
    if (interval.end_dist_on_traj < ego_dist_from_traj_front) {
      return true;
    }
    auto point_of_curr_traj = aw_ref_traj.compute(interval.end_dist_on_traj);
    const auto & prev_pose = interval.end.pose;
    const auto & curr_pose = point_of_curr_traj.pose;
    if (
      const auto is_shifted_opt =
        utils::is_point_shifted(prev_pose, curr_pose, th_pt_shift_dist_m, th_pt_shift_angle_rad)) {
      return true;
    }
    return false;
  });
}

void check_departure_points_between_intervals(
  DepartureIntervals & departure_intervals, DeparturePoints & departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double vehicle_length_m,
  const SideKey side_key)
{
  // check if departure point is in between any intervals.
  // if close to end pose, update end pose.
  for (auto & departure_interval : departure_intervals) {
    if (departure_interval.side_key != side_key) {
      continue;
    }

    for (auto & departure_point : departure_points) {
      if (departure_point.can_be_removed) {
        continue;
      }

      if (
        departure_point.dist_on_traj >= departure_interval.start_dist_on_traj &&
        departure_point.dist_on_traj <= departure_interval.end_dist_on_traj) {
        departure_point.can_be_removed = true;
        continue;
      }

      if (departure_interval.end_dist_on_traj - departure_point.dist_on_traj <= vehicle_length_m) {
        departure_interval.end = aw_ref_traj.compute(departure_point.dist_on_traj);
        departure_interval.end_dist_on_traj = departure_point.dist_on_traj;
        departure_point.can_be_removed = true;
        if (departure_point.departure_type == DepartureType::CRITICAL_DEPARTURE) {
          break;
        }
      }
    }
  }
}

void update_departure_intervals(
  DepartureIntervals & departure_intervals, Side<DeparturePoints> & departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double vehicle_length_m,
  const TrajectoryPoint & ref_traj_fr_pt, const double ego_dist_from_traj_front,
  const double th_pt_shift_dist_m, const double th_pt_shift_angle_rad)
{
  update_departure_intervals_poses(
    departure_intervals, aw_ref_traj, ref_traj_fr_pt, ego_dist_from_traj_front, th_pt_shift_dist_m,
    th_pt_shift_angle_rad);

  for (const auto side_key : g_side_keys) {
    check_departure_points_between_intervals(
      departure_intervals, departure_points[side_key], aw_ref_traj, vehicle_length_m, side_key);
  }

  if (!departure_intervals.empty()) {
    DepartureIntervals merged;
    merged.push_back(departure_intervals.front());

    for (size_t i = 1; i < departure_intervals.size(); ++i) {
      auto & next_interval_mut = departure_intervals[i];
      auto & curr_interval_mut = merged.back();
      const auto is_same_direction = curr_interval_mut.side_key == next_interval_mut.side_key;
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

void update_critical_departure_points(
  const Side<DeparturePoints> & new_departure_points,
  CriticalDeparturePoints & critical_departure_points,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double th_dist_hysteresis_m,
  const double offset_from_ego, const double th_pt_shift_dist_m, const double th_pt_shift_angle_rad)
{
  for (auto & crit_dpt_pt_mut : critical_departure_points) {
    crit_dpt_pt_mut.dist_on_traj =
      trajectory::closest(aw_ref_traj, crit_dpt_pt_mut.point_on_prev_traj);
    if (crit_dpt_pt_mut.dist_on_traj < offset_from_ego) {
      crit_dpt_pt_mut.can_be_removed = true;
      continue;
    }

    const auto updated_point = aw_ref_traj.compute(crit_dpt_pt_mut.dist_on_traj);
    if (
      const auto is_shifted_opt = utils::is_point_shifted(
        crit_dpt_pt_mut.point_on_prev_traj.pose, updated_point.pose, th_pt_shift_dist_m,
        th_pt_shift_angle_rad)) {
      crit_dpt_pt_mut.can_be_removed = true;
    }
  }
  utils::remove_if(
    critical_departure_points, [](const DeparturePoint & pt) { return pt.can_be_removed; });

  for (const auto side_key : g_side_keys) {
    for (const auto & dpt_pt : new_departure_points[side_key]) {
      if (dpt_pt.departure_type != DepartureType::CRITICAL_DEPARTURE) {
        continue;
      }

      if (dpt_pt.can_be_removed) {
        continue;
      }

      const auto is_near_curr_pts = std::any_of(
        critical_departure_points.begin(), critical_departure_points.end(),
        [&](const CriticalDeparturePoint & crit_pt) {
          return std::abs(dpt_pt.dist_on_traj - crit_pt.dist_on_traj) < th_dist_hysteresis_m;
        });

      if (is_near_curr_pts) {
        continue;
      }

      CriticalDeparturePoint crit_pt(dpt_pt);
      crit_pt.point_on_prev_traj = aw_ref_traj.compute(crit_pt.dist_on_traj);
      critical_departure_points.push_back(crit_pt);
    }
  }
  std::sort(critical_departure_points.begin(), critical_departure_points.end());
}

std::vector<std::tuple<Pose, Pose, double>> get_slow_down_intervals(
  const trajectory::Trajectory<TrajectoryPoint> & ref_traj_pts,
  const DepartureIntervals & departure_intervals,
  const SlowDownInterpolator & slow_down_interpolator, const VehicleInfo & vehicle_info,
  [[maybe_unused]] const BoundarySideWithIdx & boundary_segments, const double curr_vel,
  const double ego_dist_on_traj_m)
{
  std::vector<std::tuple<Pose, Pose, double>> slowdown_intervals;

  for (auto && pair : departure_intervals | ranges::views::enumerate) {
    const auto & [idx, departure_interval] = pair;

    const auto [slow_down_pt_on_traj, slow_down_dist_on_traj_m] =
      (ego_dist_on_traj_m < departure_interval.start_dist_on_traj)
        ? std::make_pair(
            utils::to_pt2d(departure_interval.start.pose.position),
            departure_interval.start_dist_on_traj)
        : std::make_pair(
            utils::to_pt2d(departure_interval.end.pose.position),
            departure_interval.end_dist_on_traj);

    const auto lon_dist_to_bound_m = slow_down_dist_on_traj_m - ego_dist_on_traj_m;
    // auto lat_dist_to_bound_m =
    //   boundary_departure_checker::utils::get_nearest_boundary_segment_from_point(
    //     boundary_segments[departure_interval.side_key], slow_down_pt_on_traj);

    const auto & candidates = departure_interval.candidates;
    const auto lat_dist_to_bound_itr = std::min_element(
      candidates.begin(), candidates.end(),
      [](const DeparturePoint & pt1, const DeparturePoint & pt2) {
        return pt1.lat_dist_to_bound < pt2.lat_dist_to_bound;
      });

    if (lat_dist_to_bound_itr == candidates.end()) {
      continue;
    }

    const auto lat_dist_to_bound_m = lat_dist_to_bound_itr->lat_dist_to_bound;

    const auto vel_opt = slow_down_interpolator.get_interp_to_point(
      curr_vel, lon_dist_to_bound_m, lat_dist_to_bound_m, departure_interval.side_key);

    if (!vel_opt) {
      continue;
    }

    const auto dist_to_departure_point =
      (departure_interval.start_dist_on_traj >
       (ego_dist_on_traj_m + vehicle_info.max_longitudinal_offset_m))
        ? departure_interval.start_dist_on_traj
        : departure_interval.end_dist_on_traj;

    if (ego_dist_on_traj_m >= dist_to_departure_point) {
      continue;
    }

    const auto [rel_dist_m, vel, accel_mps2] = *vel_opt;

    auto end_pose = (departure_interval.start_dist_on_traj >
                     (ego_dist_on_traj_m + vehicle_info.max_longitudinal_offset_m))
                      ? departure_interval.start.pose
                      : departure_interval.end.pose;
    if (ego_dist_on_traj_m + rel_dist_m > ref_traj_pts.length()) {
      continue;
    }

    auto start_pose = ref_traj_pts.compute(ego_dist_on_traj_m + rel_dist_m);

    slowdown_intervals.emplace_back(start_pose.pose, end_pose, vel);
  }

  return slowdown_intervals;
}

std::optional<std::pair<double, double>> is_point_shifted(
  const Pose & prev_iter_pt, const Pose & curr_iter_pt, const double th_shift_m,
  const double th_yaw_diff_rad)
{
  const auto curr_pt_yaw_rad = tf2::getYaw(curr_iter_pt.orientation);
  const auto prev_pt_yaw_rad = tf2::getYaw(prev_iter_pt.orientation);
  const auto yaw_diff_rad = std::abs(curr_pt_yaw_rad - prev_pt_yaw_rad);

  const auto dist_m = autoware_utils::calc_distance2d(curr_iter_pt.position, prev_iter_pt.position);
  if (dist_m > th_shift_m || yaw_diff_rad > th_yaw_diff_rad) {
    return std::make_pair(dist_m, yaw_diff_rad);
  }
  return std::nullopt;
}

}  // namespace autoware::motion_velocity_planner::utils
