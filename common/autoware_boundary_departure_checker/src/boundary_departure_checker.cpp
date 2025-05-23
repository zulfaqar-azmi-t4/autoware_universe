// Copyright 2020 Tier IV, Inc.
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

#include "autoware/boundary_departure_checker/boundary_departure_checker.hpp"

#include "autoware/boundary_departure_checker/utils.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <tl_expected/expected.hpp>

#include <boost/geometry.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_utils::Point2d;
using geometry_msgs::msg::Point;

bool isInAnyLane(const lanelet::ConstLanelets & candidate_lanelets, const Point2d & point)
{
  return std::any_of(
    candidate_lanelets.begin(), candidate_lanelets.end(), [&](const lanelet::ConstLanelet & ll) {
      return boost::geometry::within(point, ll.polygon2d().basicPolygon());
    });
}

}  // namespace

namespace autoware::boundary_departure_checker
{
BoundaryDepartureChecker::BoundaryDepartureChecker(
  lanelet::LaneletMapPtr lanelet_map_ptr, const VehicleInfo & vehicle_info, const Param & param,
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
: lanelet_map_ptr_(lanelet_map_ptr),
  param_ptr_(std::make_unique<Param>(param)),
  vehicle_info_ptr_(std::make_shared<VehicleInfo>(vehicle_info)),
  time_keeper_(std::move(time_keeper))
{
  auto try_uncrossable_boundaries_rtree = build_uncrossable_boundaries_tree(lanelet_map_ptr);

  if (!try_uncrossable_boundaries_rtree) {
    throw std::runtime_error(try_uncrossable_boundaries_rtree.error());
  }

  uncrossable_boundaries_rtree_ptr_ =
    std::make_unique<UncrossableBoundRTree>(*try_uncrossable_boundaries_rtree);
}

tl::expected<BDCData, std::string>
BoundaryDepartureChecker::get_projections_to_closest_uncrossable_boundaries(
  const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov, const double curr_vel,
  const TrajectoryPoints & ego_pred_traj, const double uncertainty_fp_margin_scale,
  const SteeringReport & current_steering)
{
  if (ego_pred_traj.size() > 1) {
    [[maybe_unused]] auto trajectory =
      autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(
        ego_pred_traj);
    [[maybe_unused]] const auto temp =
      experimental::trajectory::closest(*trajectory, curr_pose_with_cov.pose.position);
  }
  BDCData bdc_data;
  const auto uncertainty_fp_margin =
    utils::calc_extra_margin_from_pose_covariance(curr_pose_with_cov, uncertainty_fp_margin_scale);
  const auto ab_enveloped_footprints = utils::create_vehicle_footprints(
    ego_pred_traj, *vehicle_info_ptr_, uncertainty_fp_margin + param_ptr_->footprint_envelop);

  FootprintMargin lon_tracking_margin = uncertainty_fp_margin;
  lon_tracking_margin.lon_m = lon_tracking_margin.lon_m +
                              (curr_vel * param_ptr_->lon_tracking.scale) +
                              param_ptr_->lon_tracking.extra_margin_m;

  auto ab_lon_tracking =
    utils::create_vehicle_footprints(ego_pred_traj, *vehicle_info_ptr_, lon_tracking_margin);
  const auto ab_steering_footprints =
    utils::create_vehicle_footprints(ego_pred_traj, *vehicle_info_ptr_, current_steering);
  if (ab_enveloped_footprints.size() != ego_pred_traj.size()) {
    return tl::unexpected<std::string>("Mismatch footprint and predicted trajectory size.");
  }

  FootprintWithPose ab_enveloped_fp;
  FootprintWithPose ab_lon_tracking_fp;

  FootprintWithPose ab_steering_fp;
  ab_enveloped_fp.reserve(ab_enveloped_footprints.size());
  for (size_t i = 0; i < ab_enveloped_footprints.size(); ++i) {
    ab_enveloped_fp.emplace_back(ab_enveloped_footprints[i], ego_pred_traj[i].pose);
    ab_lon_tracking_fp.emplace_back(ab_lon_tracking[i], ego_pred_traj[i].pose);
  }
  ab_steering_fp.emplace_back(ab_steering_footprints.front(), ego_pred_traj.front().pose);

  bdc_data.ab_enveloped_fp = std::move(ab_enveloped_fp);
  bdc_data.ab_lon_tracking_fp = std::move(ab_lon_tracking_fp);
  bdc_data.ab_steering_fp = std::move(ab_steering_fp);

  if (!uncrossable_boundaries_rtree_ptr_) {
    return tl::unexpected<std::string>("No rtree available.");
  }

  if (!lanelet_map_ptr_) {
    return tl::unexpected<std::string>("lanelet_map_ptr is null");
  }

  bdc_data.ego_sides_from_footprints =
    utils::get_ego_sides_from_footprints(bdc_data.ab_enveloped_fp);

  const auto & linestring_layer = lanelet_map_ptr_->lineStringLayer;

  bdc_data.boundary_segments = utils::get_boundary_segments_from_side(
    *uncrossable_boundaries_rtree_ptr_, linestring_layer, bdc_data.ego_sides_from_footprints,
    param_ptr_->th_max_lateral_query_num);

  bdc_data.side_to_bound_projections = utils::get_closest_boundary_segments_from_side(
    bdc_data.boundary_segments, bdc_data.ego_sides_from_footprints);

  return bdc_data;
}

tl::expected<UncrossableBoundRTree, std::string>
BoundaryDepartureChecker::build_uncrossable_boundaries_tree(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!lanelet_map_ptr) {
    return tl::unexpected(std::string("lanelet_map_ptr is null"));
  }

  if (!param_ptr_) {
    return tl::unexpected(std::string("param_ptr is null"));
  }

  return utils::build_uncrossable_boundaries_rtree(
    *lanelet_map_ptr, param_ptr_->boundary_types_to_detect);
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  lanelet::ConstLanelets candidate_lanelets =
    utils::getCandidateLanelets(lanelets, vehicle_footprints);
  return willLeaveLane(candidate_lanelets, vehicle_footprints);
}

bool BoundaryDepartureChecker::willLeaveLane(
  const lanelet::ConstLanelets & candidate_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & vehicle_footprint : vehicle_footprints) {
    if (isOutOfLane(candidate_lanelets, vehicle_footprint)) {
      return true;
    }
  }

  return false;
}

std::vector<std::pair<double, lanelet::Lanelet>> BoundaryDepartureChecker::getLaneletsFromPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Get Footprint Hull basic polygon
  std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  LinearRing2d footprint_hull = utils::createHullFromFootprints(vehicle_footprints);

  lanelet::BasicPolygon2d footprint_hull_basic_polygon = utils::toBasicPolygon2D(footprint_hull);

  // Find all lanelets that intersect the footprint hull
  return lanelet::geometry::findWithin2d(
    lanelet_map_ptr->laneletLayer, footprint_hull_basic_polygon, 0.0);
}

std::optional<autoware_utils::Polygon2d> BoundaryDepartureChecker::getFusedLaneletPolygonForPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto lanelets_distance_pair = getLaneletsFromPath(lanelet_map_ptr, path);
  if (lanelets_distance_pair.empty()) return std::nullopt;

  // Fuse lanelets into a single BasicPolygon2d
  autoware_utils::MultiPolygon2d lanelet_unions;
  autoware_utils::MultiPolygon2d result;

  for (size_t i = 0; i < lanelets_distance_pair.size(); ++i) {
    const auto & route_lanelet = lanelets_distance_pair.at(i).second;
    const auto & p = route_lanelet.polygon2d().basicPolygon();
    autoware_utils::Polygon2d poly = toPolygon2D(p);
    boost::geometry::union_(lanelet_unions, poly, result);
    lanelet_unions = result;
    result.clear();
  }

  if (lanelet_unions.empty()) return std::nullopt;
  return lanelet_unions.front();
}

bool BoundaryDepartureChecker::updateFusedLaneletPolygonForPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto lanelets_distance_pair = getLaneletsFromPath(lanelet_map_ptr, path);
  if (lanelets_distance_pair.empty()) return false;

  autoware_utils::MultiPolygon2d lanelet_unions;
  autoware_utils::MultiPolygon2d result;

  if (fused_lanelets_polygon) lanelet_unions.push_back(fused_lanelets_polygon.value());

  for (size_t i = 0; i < lanelets_distance_pair.size(); ++i) {
    const auto & route_lanelet = lanelets_distance_pair.at(i).second;
    bool id_exist = std::any_of(
      fused_lanelets_id.begin(), fused_lanelets_id.end(),
      [&](const auto & id) { return id == route_lanelet.id(); });

    if (id_exist) continue;

    const auto & p = route_lanelet.polygon2d().basicPolygon();
    autoware_utils::Polygon2d poly = toPolygon2D(p);
    boost::geometry::union_(lanelet_unions, poly, result);
    lanelet_unions = result;
    result.clear();
    fused_lanelets_id.push_back(route_lanelet.id());
  }

  if (lanelet_unions.empty()) {
    fused_lanelets_polygon = std::nullopt;
    return false;
  }

  fused_lanelets_polygon = lanelet_unions.front();
  return true;
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // check if the footprint is not fully contained within the fused lanelets polygon
  const std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  const auto fused_lanelets_polygon = getFusedLaneletPolygonForPath(lanelet_map_ptr, path);
  if (!fused_lanelets_polygon) return true;
  return !std::all_of(
    vehicle_footprints.begin(), vehicle_footprints.end(),
    [&fused_lanelets_polygon](const auto & footprint) {
      return boost::geometry::within(footprint, fused_lanelets_polygon.value());
    });
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);

  auto is_all_footprints_within = [&](const auto & polygon) {
    return std::all_of(
      vehicle_footprints.begin(), vehicle_footprints.end(),
      [&polygon](const auto & footprint) { return boost::geometry::within(footprint, polygon); });
  };

  // If lanelets polygon exists and all footprints are within it, the path doesn't leave the lane
  if (fused_lanelets_polygon && is_all_footprints_within(fused_lanelets_polygon.value())) {
    return false;
  }

  // Update the lanelet polygon for the current path
  if (!updateFusedLaneletPolygonForPath(
        lanelet_map_ptr, path, fused_lanelets_id, fused_lanelets_polygon)) {
    // If update fails, assume the path leaves the lane
    return true;
  }

  // Check if any footprint is outside the updated lanelets polygon
  return !is_all_footprints_within(fused_lanelets_polygon.value());
}

PathWithLaneId BoundaryDepartureChecker::cropPointsOutsideOfLanes(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path, const size_t end_index,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  PathWithLaneId temp_path;

  // Update the lanelet polygon for the current path
  if (
    path.points.empty() || !updateFusedLaneletPolygonForPath(
                             lanelet_map_ptr, path, fused_lanelets_id, fused_lanelets_polygon)) {
    return temp_path;
  }

  const auto vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);

  {
    autoware_utils::ScopedTimeTrack st2(
      "check if footprint is within fused_lanelets_polygon", *time_keeper_);

    size_t idx = 0;
    std::for_each(
      vehicle_footprints.begin(), vehicle_footprints.end(), [&](const auto & footprint) {
        if (idx > end_index || boost::geometry::within(footprint, fused_lanelets_polygon.value())) {
          temp_path.points.push_back(path.points.at(idx));
        }
        ++idx;
      });
  }
  PathWithLaneId cropped_path = path;
  cropped_path.points = temp_path.points;
  return cropped_path;
}

bool BoundaryDepartureChecker::isOutOfLane(
  const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : vehicle_footprint) {
    if (!isInAnyLane(candidate_lanelets, point)) {
      return true;
    }
  }

  return false;
}

SegmentRtree BoundaryDepartureChecker::extractUncrossableBoundaries(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Point & ego_point,
  const double max_search_length, const std::vector<std::string> & boundary_types_to_detect)
{
  const auto has_types =
    [](const lanelet::ConstLineString3d & ls, const std::vector<std::string> & types) {
      constexpr auto no_type = "";
      const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
      return (type != no_type && std::find(types.begin(), types.end(), type) != types.end());
    };

  SegmentRtree uncrossable_segments_in_range;
  LineString2d line;
  const auto ego_p = Point2d{ego_point.x, ego_point.y};
  for (const auto & ls : lanelet_map.lineStringLayer) {
    if (has_types(ls, boundary_types_to_detect)) {
      line.clear();
      for (const auto & p : ls) line.push_back(Point2d{p.x(), p.y()});
      for (auto segment_idx = 0LU; segment_idx + 1 < line.size(); ++segment_idx) {
        const Segment2d segment = {line[segment_idx], line[segment_idx + 1]};
        if (boost::geometry::distance(segment, ego_p) < max_search_length) {
          uncrossable_segments_in_range.insert(segment);
        }
      }
    }
  }
  return uncrossable_segments_in_range;
}

autoware_utils::Polygon2d BoundaryDepartureChecker::toPolygon2D(
  const lanelet::BasicPolygon2d & poly) const
{
  autoware_utils::Polygon2d polygon;
  auto & outer = polygon.outer();

  for (const auto & p : poly) {
    autoware_utils::Point2d p2d(p.x(), p.y());
    outer.push_back(p2d);
  }
  boost::geometry::correct(polygon);
  return polygon;
}

}  // namespace autoware::boundary_departure_checker
