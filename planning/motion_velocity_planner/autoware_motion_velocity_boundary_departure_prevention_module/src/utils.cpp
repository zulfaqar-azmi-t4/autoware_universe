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

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
param::DepartureTypeesIdx check_departure_status(
  const SideToBoundPojections & side_to_bound_projections, const param::NodeParam & param)
{
  param::DepartureTypeesIdx stats;
  for (const auto & left : side_to_bound_projections.left) {
    const auto & [projection, departed_segment, idx_from_orig] = left;
    const auto & [p_orig, p_proj, dist] = projection;
    if (std::abs(dist) < param.stop_before_departure.th_dist_to_boundary_m.left) {
      stats.left.emplace_back(DepartureType::CRITICAL_DEPARTURE, idx_from_orig);
    }

    if (std::abs(dist) < param.slow_down_before_departure.th_dist_to_boundary_m.left) {
      stats.left.emplace_back(DepartureType::APPROACHING_DEPARTURE, idx_from_orig);
    }

    if (std::abs(dist) < param.slow_down_near_boundary.th_dist_to_boundary_m.left) {
      stats.left.emplace_back(DepartureType::NEAR_BOUNDARY, idx_from_orig);
    }
  }

  for (const auto & right : side_to_bound_projections.right) {
    const auto & [projection, departed_segment, idx_from_orig] = right;
    const auto & [p_orig, p_proj, dist] = projection;
    if (std::abs(dist) < param.stop_before_departure.th_dist_to_boundary_m.right) {
      stats.right.emplace_back(DepartureType::CRITICAL_DEPARTURE, idx_from_orig);
    }

    if (std::abs(dist) < param.slow_down_before_departure.th_dist_to_boundary_m.right) {
      stats.right.emplace_back(DepartureType::APPROACHING_DEPARTURE, idx_from_orig);
    }

    if (std::abs(dist) < param.slow_down_near_boundary.th_dist_to_boundary_m.right) {
      stats.right.emplace_back(DepartureType::NEAR_BOUNDARY, idx_from_orig);
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
}  // namespace autoware::motion_velocity_planner::utils
