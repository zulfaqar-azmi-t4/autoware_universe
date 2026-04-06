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

#include "autoware/boundary_departure_checker/uncrossable_boundary_checker.hpp"

#include "autoware/boundary_departure_checker/conversion.hpp"
#include "autoware/boundary_departure_checker/footprints_generator.hpp"
#include "autoware/boundary_departure_checker/utils.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>
#include <tf2/utils.hpp>
#include <tl_expected/expected.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
namespace bg = boost::geometry;

/**
 * @brief Retrieves a 3D line segment from the Lanelet2 map.
 *
 * @param lanelet_map_ptr A pointer to the Lanelet2 map from which to retrieve the data.
 * @param seg_id An identifier struct containing the ID of the parent LineString and the start/end
 * indices of the specific segment within it.
 * @return The corresponding Segment3d defined by the start and end points.
 */
autoware_utils_geometry::Segment3d get_segment_3d_from_id(
  const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const autoware::boundary_departure_checker::IdxForRTreeSegment & seg_id)
{
  const auto & linestring_layer = lanelet_map_ptr->lineStringLayer;
  const auto basic_ls = linestring_layer.get(seg_id.linestring_id).basicLineString();

  auto p_start = autoware_utils_geometry::Point3d{
    basic_ls.at(seg_id.segment_start_idx).x(), basic_ls.at(seg_id.segment_start_idx).y(),
    basic_ls.at(seg_id.segment_start_idx).z()};

  auto p_end = autoware_utils_geometry::Point3d{
    basic_ls.at(seg_id.segment_end_idx).x(), basic_ls.at(seg_id.segment_end_idx).y(),
    basic_ls.at(seg_id.segment_end_idx).z()};

  return {p_start, p_end};
}

/**
 * @brief Checks if a given boundary segment is closer to the reference ego side than the opposite
 * side.
 *
 * @param boundary_segment The boundary segment to check.
 * @param ego_side_ref_segment The reference side of the ego vehicle (e.g., the left side).
 * @param ego_side_opposite_ref_segment The opposite side of the ego vehicle (e.g., the right side).
 * @return True if the boundary is closer to or equidistant to the reference side; false otherwise.
 */
bool is_closest_to_boundary_segment(
  const autoware_utils_geometry::Segment2d & boundary_segment,
  const autoware_utils_geometry::Segment2d & ego_side_ref_segment,
  const autoware_utils_geometry::Segment2d & ego_side_opposite_ref_segment)
{
  const auto dist_from_curr_side = bg::comparable_distance(ego_side_ref_segment, boundary_segment);
  const auto dist_from_compare_side =
    bg::comparable_distance(ego_side_opposite_ref_segment, boundary_segment);

  return dist_from_curr_side <= dist_from_compare_side;
}

/**
 * @brief Checks if a 3D boundary segment is vertically within the height range of the ego vehicle.
 *
 * This helps filter out irrelevant boundaries like overpasses (too high) or underpass (too low).
 *
 * @param boundary_segment The 3D boundary segment to check.
 * @param ego_z_position The reference vertical (Z-axis) position of the ego vehicle (e.g., at its
 * base).
 * @param ego_height The total height of the ego vehicle.
 * @return True if the segment's closest vertical point is within the vehicle's height; false
 * otherwise.
 */
bool is_segment_within_ego_height(
  const autoware_utils_geometry::Segment3d & boundary_segment, const double ego_z_position,
  const double ego_height)
{
  auto height_diff = std::min(
    std::abs(boundary_segment.first.z() - ego_z_position),
    std::abs(boundary_segment.second.z() - ego_z_position));
  return height_diff < ego_height;
}
}  // namespace

namespace autoware::boundary_departure_checker
{
UncrossableBoundaryChecker::UncrossableBoundaryChecker(
  const rclcpp::Clock::SharedPtr clock_ptr, lanelet::LaneletMapPtr lanelet_map_ptr)
: clock_ptr_(clock_ptr), lanelet_map_ptr_(lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    throw std::runtime_error("lanelet_map_ptr is null");
  }
  auto try_uncrossable_boundaries_rtree = build_uncrossable_boundaries_tree(lanelet_map_ptr);

  if (!try_uncrossable_boundaries_rtree) {
    throw std::runtime_error(try_uncrossable_boundaries_rtree.error());
  }

  uncrossable_boundaries_rtree_ptr_ =
    std::make_unique<UncrossableBoundsRTree>(*try_uncrossable_boundaries_rtree);
}

void UncrossableBoundaryChecker::set_param(const UncrossableBoundaryDepartureParam & param)
{
  param_ = param;
}

tl::expected<UncrossableBoundsRTree, std::string>
UncrossableBoundaryChecker::build_uncrossable_boundaries_tree(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!lanelet_map_ptr) {
    return tl::make_unexpected("lanelet_map_ptr is null");
  }

  return utils::build_uncrossable_boundaries_rtree(
    *lanelet_map_ptr, param_.boundary_types_to_detect);
}

bool is_critical(const Side<ProjectionsToBound> & evaluated_projections)
{
  const auto check_side_for_critical = [&](const ProjectionsToBound & side_value) {
    return std::any_of(
      side_value.rbegin(), side_value.rend(), [](const auto & pt) { return pt.is_critical(); });
  };

  return evaluated_projections.any_of_side(check_side_for_critical);
}
tl::expected<DepartureData, std::string> UncrossableBoundaryChecker::check_departure(
  const TrajectoryPoints & predicted_traj, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const EgoDynamicState & ego_state)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (predicted_traj.empty()) {
    return {};
  }

  DepartureData departure_data;
  departure_data.footprints =
    footprints::generate(predicted_traj, vehicle_info, ego_state.pose_with_cov);

  if (predicted_traj.size() != departure_data.footprints.size()) {
    return tl::make_unexpected(
      "Size of generated footprints does not match size of predicted trajectory");
  }

  departure_data.footprints_sides =
    footprints::get_sides_from_footprints(departure_data.footprints);

  if (departure_data.footprints.size() != departure_data.footprints_sides.size()) {
    return tl::make_unexpected(
      "Size of generated footprints does not match size of predicted trajectory");
  }

  departure_data.boundary_segments = get_boundary_segments(
    departure_data.footprints_sides, predicted_traj, vehicle_info.vehicle_height_m);

  if (departure_data.boundary_segments.all_empty()) {
    return {};
  }

  departure_data.projections_to_bound = utils::get_closest_boundary_segments_from_side(
    predicted_traj, departure_data.boundary_segments, departure_data.footprints_sides);

  departure_data.evaluated_projections = evaluate_projections_across_sides(
    departure_data.projections_to_bound, ego_state.velocity, ego_state.acceleration);

  std::invoke(
    [&](const auto & evaluated_projections) {
      if (!is_critical_departure_persist(evaluated_projections)) {
        critical_departure_.for_each_side([](auto & side) { side.clear(); });
        return;
      }

      if (is_continuous_critical_departure(evaluated_projections)) {
        return;
      }

      evaluated_projections.for_each([&](auto key_constant, auto & side_value) {
        constexpr SideKey side_key = key_constant.value;
        for (const auto & proj : side_value) {
          if (proj.is_critical()) {
            critical_departure_[side_key].push_back(proj);
          }
        }
      });
    },
    departure_data.evaluated_projections);

  departure_data.status = critical_departure_.any_of_side([](const auto & side_value) {
    return std::any_of(
      side_value.begin(), side_value.end(), [](const auto & proj) { return proj.is_critical(); });
  })
                            ? DepartureType::CRITICAL
                            : DepartureType::NONE;

  return departure_data;
}

std::vector<SegmentWithIdx> UncrossableBoundaryChecker::find_closest_boundary_segments(
  const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
  const double ego_z_position, const double ego_vehicle_height,
  const std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id) const
{
  if (!lanelet_map_ptr_ || !uncrossable_boundaries_rtree_ptr_) {
    return {};
  }

  const auto & rtree = *uncrossable_boundaries_rtree_ptr_;
  const lanelet::BasicPoint2d ego_start{ego_ref_segment.first.x(), ego_ref_segment.first.y()};

  std::vector<SegmentWithIdx> nearest_raw;
  rtree.query(
    bgi::nearest(ego_start, param_.max_lateral_rtree_queries), std::back_inserter(nearest_raw));

  std::vector<SegmentWithIdx> new_segments;
  for (const auto & nearest : nearest_raw) {
    const auto & id = nearest.second;
    if (unique_id.find(id) != unique_id.end()) {
      continue;  // Skip if this segment has already been added
    }

    auto boundary_segment_3d = get_segment_3d_from_id(lanelet_map_ptr_, id);

    if (!is_segment_within_ego_height(boundary_segment_3d, ego_z_position, ego_vehicle_height)) {
      continue;
    }

    auto boundary_segment = utils::to_segment_2d(boundary_segment_3d);

    if (is_closest_to_boundary_segment(
          boundary_segment, ego_ref_segment, ego_opposite_ref_segment)) {
      new_segments.emplace_back(boundary_segment, id);
    }
  }
  return new_segments;
}

BoundarySegmentsBySide UncrossableBoundaryChecker::get_boundary_segments(
  const FootprintSideSegmentsArray & footprints_sides,
  const TrajectoryPoints & trimmed_pred_trajectory, const double ego_vehicle_height) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  BoundarySegmentsBySide boundary_sides_with_idx;
  std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> unique_ids;

  for (const auto & [fp, traj_pt] : ranges::views::zip(footprints_sides, trimmed_pred_trajectory)) {
    const auto ego_z = traj_pt.pose.position.z;

    auto left_segs =
      find_closest_boundary_segments(fp.left, fp.right, ego_z, ego_vehicle_height, unique_ids);
    for (auto & seg : left_segs) {
      unique_ids.insert(seg.second);
      boundary_sides_with_idx.left.emplace_back(std::move(seg));
    }

    auto right_segs =
      find_closest_boundary_segments(fp.right, fp.left, ego_z, ego_vehicle_height, unique_ids);
    for (auto & seg : right_segs) {
      unique_ids.insert(seg.second);
      boundary_sides_with_idx.right.emplace_back(std::move(seg));
    }
  }
  return boundary_sides_with_idx;
}

Side<ProjectionsToBound> UncrossableBoundaryChecker::evaluate_projections_across_sides(
  const Side<ProjectionsToBound> & projections_to_bound, const double curr_vel,
  const double curr_acc) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto min_braking_dist_opt = motion_utils::calculate_stop_distance(
    curr_vel, curr_acc, param_.max_deceleration_mps2, param_.max_jerk_mps3, param_.brake_delay_s);

  if (!min_braking_dist_opt) {
    return {};
  }

  Side<ProjectionsToBound> min_to_bound =
    utils::evaluate_projections_severity(projections_to_bound, param_, *min_braking_dist_opt);

  return min_to_bound;
}

bool UncrossableBoundaryChecker::is_continuous_critical_departure(
  const Side<ProjectionsToBound> & evaluated_projections)
{
  const auto is_critical_departure_detected = is_critical(evaluated_projections);

  if (!is_critical_departure_detected) {
    last_no_critical_dpt_time_ = clock_ptr_->now().seconds();
    return false;
  }

  const auto t_diff = clock_ptr_->now().seconds() - last_no_critical_dpt_time_;
  return t_diff >= param_.on_time_buffer_s;
}

bool UncrossableBoundaryChecker::is_critical_departure_persist(
  const Side<ProjectionsToBound> & evaluated_projections)
{
  const auto is_critical_departure_detected =
    is_critical(evaluated_projections) && !critical_departure_.all_empty();

  if (is_critical_departure_detected) {
    last_found_critical_dpt_time_ = clock_ptr_->now().seconds();
    return true;
  }

  const auto t_diff = clock_ptr_->now().seconds() - last_found_critical_dpt_time_;
  return t_diff >= param_.off_time_buffer_s;
}

UncrossableBoundaryChecker::~UncrossableBoundaryChecker() = default;
}  // namespace autoware::boundary_departure_checker
