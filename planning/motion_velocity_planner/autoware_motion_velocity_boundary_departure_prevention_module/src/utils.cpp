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

#include <magic_enum.hpp>

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
  [[maybe_unused]] const EgoSides & ego_sides,
  const SideToBoundPojections & side_to_bound_projections, const param::NodeParam & param,
  [[maybe_unused]] const double curr_vel)
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
double compute_braking_distance(double v_init, double v_end, double a, double j)
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

  return d1 + d2;
}

}  // namespace autoware::motion_velocity_planner::utils
