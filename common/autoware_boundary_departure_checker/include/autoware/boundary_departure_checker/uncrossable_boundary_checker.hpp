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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::boundary_departure_checker
{

class FootprintManager;

class UncrossableBoundaryChecker
{
public:
  UncrossableBoundaryChecker(
    const rclcpp::Clock::SharedPtr clock_ptr, lanelet::LaneletMapPtr lanelet_map_ptr);

  UncrossableBoundaryChecker() = delete;
  UncrossableBoundaryChecker(const UncrossableBoundaryChecker &) = delete;
  UncrossableBoundaryChecker(UncrossableBoundaryChecker &&) = delete;
  UncrossableBoundaryChecker & operator=(const UncrossableBoundaryChecker &) = delete;
  UncrossableBoundaryChecker & operator=(UncrossableBoundaryChecker &&) = delete;
  ~UncrossableBoundaryChecker();

  void set_param(const UncrossableBoundaryDepartureParam & param);

  /**
   * @brief Generate data structure with embedded abnormality information based on the
   * predicted trajectory and current ego state.
   *
   * This function creates extended ego footprints for various abnormality types (e.g.,
   * localization, steering) and computes their corresponding closest boundary projections and
   * segments.
   *
   * @param predicted_traj         Ego's predicted trajectory (from MPC or trajectory follower).
   * @param vehicle_info           Vehicle information.
   * @param ego_state              Ego dynamic state (pose, velocity, acceleration).
   * @return DepartureData containing footprints, their left/right sides, and projections to
   * boundaries. Returns an error message string on failure.
   */
  tl::expected<DepartureData, std::string> check_departure(
    const TrajectoryPoints & predicted_traj, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const EgoDynamicState & ego_state) const;

  /**
   * @brief Queries a spatial index (R-tree) to find nearby uncrossable lane boundaries and filters
   * them.
   *
   * @param ego_ref_segment The reference side of the ego vehicle (e.g., the left side) used as the
   * query origin.
   * @param ego_opposite_ref_segment The opposite side of the ego vehicle (e.g., the right side),
   * used to filter out boundaries on the wrong side.
   * @param ego_z_position The current vertical (Z-axis) position of the ego vehicle, used to filter
   * boundaries by height.
   * @param unique_id A set of segment IDs that have already been processed, used to avoid adding
   * duplicate boundaries.
   * @return A vector of `SegmentWithIdx` containing the filtered boundary segments that are deemed
   * relevant and close to the reference side.
   */
  [[nodiscard]] std::vector<SegmentWithIdx> find_closest_boundary_segments(
    const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
    const double ego_z_position, const double ego_vehicle_height,
    const std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id) const;

  /**
   * @brief Collects all relevant uncrossable boundary segments along a predicted trajectory.
   *
   * @param footprints_sides A container of the vehicle's left and right side segments for
   * each point along the trajectory.
   * @param trimmed_pred_trajectory The predicted trajectory of the ego vehicle, used to get the
   * Z-position at each step.
   * @return A `BoundarySegmentsBySide` struct containing two vectors: one for all unique, relevant
   * boundaries found to the left of the trajectory, and one for the right.
   */
  [[nodiscard]] BoundarySegmentsBySide get_boundary_segments(
    const FootprintSideSegmentsArray & footprints_sides,
    const TrajectoryPoints & trimmed_pred_trajectory, const double ego_vehicle_height) const;

  /**
   * @brief Select the closest projections to boundaries for both sides based on all abnormality
   * types.
   *
   * Invokes `evaluate_projections_severity` for each side.
   *
   * @param projections_to_bound Footprint sides' projections to boundaries.
   * @return Side<ProjectionsToBound> structure containing selected points for both sides.
   */
  [[nodiscard]] Side<ProjectionsToBound> evaluate_projections_across_sides(
    const Side<ProjectionsToBound> & projections_to_bound, const double curr_vel,
    const double curr_acc) const;

private:
  // Member variables
  UncrossableBoundaryDepartureParam param_;
  rclcpp::Clock::SharedPtr clock_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::unique_ptr<UncrossableBoundsRTree> uncrossable_boundaries_rtree_ptr_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  // Private member functions
  /**
   * @brief Build an R-tree of uncrossable boundaries (e.g., road_border) from a lanelet map.
   *
   * Filters the map's line strings by type and constructs a spatial index used to detect boundary
   * violations.
   *
   * @param lanelet_map_ptr Shared pointer to the lanelet map.
   * @return Constructed R-tree or an error message if map or parameters are invalid.
   */
  tl::expected<UncrossableBoundsRTree, std::string> build_uncrossable_boundaries_tree(
    const lanelet::LaneletMapPtr & lanelet_map_ptr);
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_
