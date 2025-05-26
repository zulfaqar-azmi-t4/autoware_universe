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

#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
DeparturePoint create_departure_point(
  const Point2d & candidate_point, const DepartureType & departure_type,
  const param::NodeParam & node_param, std::string_view direction)
{
  DeparturePoint point;
  point.uuid = autoware_utils::to_hex_string(autoware_utils::generate_uuid());
  point.point = candidate_point;
  point.direction = direction;
  point.th_dist_hysteresis = node_param.th_dist_hysteresis_m;
  point.type = departure_type;
  return point;
}

param::DepartureTypeesIdx check_departure_status(
  const EgoSides & ego_sides, const SideToBoundPojections & side_to_bound_projections,
  const param::NodeParam & param, const double curr_vel)
{
  param::DepartureTypeesIdx stats;
  const auto assign_status =
    [](const double lat_dist_m, const auto & param, const auto & side_key) -> DepartureType {
    const auto stop_before_dpt = param.stop_before_departure.th_dist_to_boundary_m[side_key];
    const auto slow_before_dpt = param.slow_down_before_departure.th_dist_to_boundary_m[side_key];
    const auto slow_near_bound = param.slow_down_near_boundary.th_dist_to_boundary_m[side_key];

    if (std::abs(lat_dist_m) < stop_before_dpt) {
      return DepartureType::CRITICAL_DEPARTURE;
    }

    if (std::abs(lat_dist_m) < slow_before_dpt) {
      return DepartureType::APPROACHING_DEPARTURE;
    }

    if (std::abs(lat_dist_m) < slow_near_bound) {
      return DepartureType::NEAR_BOUNDARY;
    }

    return DepartureType::NONE;
  };

  for (const auto side_key : side_keys) {
    for (const auto & [projection, departed_segment, idx_from_orig] :
         side_to_bound_projections[side_key]) {
      const auto & [p_orig, p_proj, lat_dist_m] = projection;
      const auto status = assign_status(lat_dist_m, param, side_key);
      if (status != DepartureType::NONE && status != DepartureType::UNKNOWN) {
        stats[side_key].emplace_back(status, idx_from_orig);
      }
    }
  }

  const auto is_far = [&](const auto & param, const auto dist_to_start) {
    const auto braking_dist = utils::calc_braking_distance(
      std::clamp(curr_vel, 5.0 / 3.6, 35.0 / 3.6), param.th_trigger.decel_mp2,
      param.th_trigger.brake_delay_s, param.th_trigger.dist_error_m);
    return (braking_dist > dist_to_start);
  };

  const auto cond = [&](const param::DepartureTypeIdx & side) {
    const auto [status, idx] = side;
    const auto dist_to_start = ego_sides.at(idx).dist_from_start;
    if (status == DepartureType::CRITICAL_DEPARTURE) {
      return is_far(param.stop_before_departure, dist_to_start);
    }

    if (status == DepartureType::APPROACHING_DEPARTURE) {
      return is_far(param.slow_down_before_departure, dist_to_start);
    }

    if (status == DepartureType::NEAR_BOUNDARY) {
      return is_far(param.slow_down_near_boundary, dist_to_start);
    }
    return true;
  };

  for (const auto side_key : side_keys) {
    auto remove_itr = std::remove_if(stats[side_key].begin(), stats[side_key].end(), cond);
    stats[side_key].erase(remove_itr, stats[side_key].end());
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
  const std::vector<param::DepartureTypeIdx> & departure_stats, const EgoSides & ego_sides,
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
}  // namespace autoware::motion_velocity_planner::utils
