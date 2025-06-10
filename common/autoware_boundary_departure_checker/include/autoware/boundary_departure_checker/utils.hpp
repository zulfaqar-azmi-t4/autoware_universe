// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_

#include "autoware/boundary_departure_checker/parameters.hpp"

#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <string>
#include <vector>

namespace autoware::boundary_departure_checker::utils
{
double calc_dist_on_traj(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const Point2d & point);
Point2d to_point2d(const Eigen::Matrix<double, 3, 1> & ll_pt);

Segment2d to_segment2d(
  const Eigen::Matrix<double, 3, 1> & ll_pt1, const Eigen::Matrix<double, 3, 1> & ll_pt2);

DepartureType check_departure_type(
  const double lateral_dist_m, const Param & param, const SideKey side_key);

std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const SteeringReport & current_steering);
std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const FootprintMargin & uncertainty_fp_margin, const LongitudinalConfig & longitudinal_config);
std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const FootprintMargin & margin = {0.0, 0.0});

std::vector<LinearRing2d> create_ego_footprints(
  const AbnormalityType abnormality_type, const FootprintMargin & uncertainty_fp_margin,
  const TrajectoryPoints & ego_pred_traj, const SteeringReport & current_steering,
  const VehicleInfo & vehicle_info, const Param & param);
/**
 * @brief cut trajectory by length
 * @param trajectory input trajectory
 * @param length cut length
 * @return cut trajectory
 */
TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length);

/**
 * @brief resample the input trajectory with the given interval
 * @param trajectory input trajectory
 * @param interval resampling interval
 * @return resampled trajectory
 * @note this function assumes the input trajectory is sampled dense enough
 */
TrajectoryPoints resampleTrajectory(const Trajectory & trajectory, const double interval);

/**
 * @brief create vehicle footprints along the trajectory with the given covariance and margin
 * @param covariance vehicle pose with covariance
 * @param trajectory trajectory along which the vehicle footprints are created
 * @param vehicle_info vehicle information
 * @param footprint_margin_scale scale of the footprint margin
 * @return vehicle footprints along the trajectory
 */
std::vector<std::pair<LinearRing2d, Pose>> createVehicleFootprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const VehicleInfo & vehicle_info, const double footprint_margin_scale);

/**
 * @brief create vehicle footprints along the path with the given margin
 * @param path path along which the vehicle footprints are created
 * @param vehicle_info vehicle information
 * @param footprint_extra_margin extra margin for the footprint
 * @return vehicle footprints along the path
 */
std::vector<LinearRing2d> createVehicleFootprints(
  const PathWithLaneId & path, const VehicleInfo & vehicle_info,
  const double footprint_extra_margin);

/**
 * @brief find lanelets that potentially intersect with the vehicle's trajectory
 * @param route_lanelets lanelets along the planned route
 * @param vehicle_footprints series of vehicle footprint polygons along the trajectory
 * @return lanelets that are not disjoint from the convex hull of vehicle footprints
 */
lanelet::ConstLanelets getCandidateLanelets(
  const lanelet::ConstLanelets & route_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints);

/**
 * @brief create a convex hull from multiple footprint polygons
 * @param footprints collection of footprint polygons represented as LinearRing2d
 * @return a single LinearRing2d representing the convex hull containing all input footprints
 */
LinearRing2d createHullFromFootprints(const std::vector<LinearRing2d> & footprints);

/**
 * @brief create passing areas of the vehicle from vehicle footprints
 * @param vehicle_footprints vehicle footprints along trajectory
 * @return passing areas of the vehicle that are created from adjacent vehicle footprints
 *         If vehicle_footprints is empty, returns empty vector
 *         If vehicle_footprints size is 1, returns vector with that footprint
 */
std::vector<LinearRing2d> createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints);

/**
 * @brief calculate the maximum search length for boundaries considering the vehicle dimensions
 * @param trajectory target trajectory
 * @param vehicle_info vehicle information
 * @return maximum search length for boundaries
 */
double calcMaxSearchLengthForBoundaries(
  const Trajectory & trajectory, const VehicleInfo & vehicle_info);

UncrossableBoundRTree build_uncrossable_boundaries_rtree(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect);

ProjectionsToBound get_closest_boundary_segments_from_side(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const BoundarySideWithIdx & boundaries, const EgoSides & ego_sides_from_footprints);

lanelet::BasicPolygon2d toBasicPolygon2D(const LinearRing2d & footprint_hull);

Side<Segment2d> get_footprint_sides(
  const LinearRing2d & footprint, const bool use_center_right, const bool use_center_left);

LineString2d to_linestring_2d(const Segment2d & segment);

double cross_2d(const Point2d & point, const Segment2d & seg);

bool is_point_left_of_line(const Point2d & point, const Segment2d & seg);

std::vector<lanelet::ConstLineString3d> get_linestrings_near_footprint(
  const lanelet::LineStringLayer & linestring_layer, const Pose & ego_pose,
  const double search_distance,
  const std::vector<std::string> uncrossable_boundary_types = {"road_border"});

FootprintMargin calc_margin_from_covariance(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale);

EgoSide get_ego_side_from_footprint(
  const Footprint & footprint, const bool use_center_right = true,
  const bool use_center_left = true);

EgoSides get_sides_from_footprints(
  const Footprints & footprints, const bool use_center_right = false,
  const bool use_center_left = false);

double compute_braking_distance(
  const double v_init, const double v_end, const double acc, const double jerk,
  double t_braking_delay);
}  // namespace autoware::boundary_departure_checker::utils

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_
