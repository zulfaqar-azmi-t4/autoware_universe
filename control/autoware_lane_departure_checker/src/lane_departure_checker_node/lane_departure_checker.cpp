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

#include "autoware/lane_departure_checker/lane_departure_checker.hpp"

#include "autoware/lane_departure_checker/utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <boost/geometry.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>
// for writing the svg file
#include <fstream>
// for the geometry types
// for the svg mapper
#include <boost/geometry/io/svg/svg_mapper.hpp>
#include <boost/geometry/io/svg/write.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using autoware_utils::LinearRing2d;
using autoware_utils::LineString2d;
using autoware_utils::MultiPoint2d;
using autoware_utils::MultiPolygon2d;
using autoware_utils::Point2d;

namespace
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

namespace bg = boost::geometry;
namespace bgi = bg::index;
using autoware_utils::Box2d;
using autoware_utils::Point2d;
using autoware_utils::Segment2d;
using geometry_msgs::msg::Point;
// R-Tree stores bounding boxes with an index reference to segments
using SegmentWithIdx = std::pair<Segment2d, size_t>;
using SegmentRtree = bgi::rtree<SegmentWithIdx, bgi::rstar<16>>;  // Use R* tree

double calcBrakingDistance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  return (abs_velocity * abs_velocity) / (2.0 * max_deceleration) + delay_time * abs_velocity;
}

bool isInAnyLane(const lanelet::ConstLanelets & candidate_lanelets, const Point2d & point)
{
  for (const auto & ll : candidate_lanelets) {
    if (boost::geometry::within(point, ll.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}

std::optional<SegmentRtree> build_rtree(const std::vector<Segment2d> & boundary_segments)
{
  std::vector<SegmentWithIdx> boxes_with_idx;
  boxes_with_idx.reserve(boundary_segments.size());
  for (size_t i = 0; i < boundary_segments.size(); ++i) {
    Segment2d segment2d;
    bg::envelope(boundary_segments[i], segment2d);
    boxes_with_idx.emplace_back(segment2d, i);
  }
  return SegmentRtree(boxes_with_idx.begin(), boxes_with_idx.end());
}

std::pair<Point2d, double> point_to_segment_signed_projection(
  const Point2d & p, const Segment2d & segment)
{
  const auto & p1 = segment.first;
  const auto & p2 = segment.second;

  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 <= 0) return {p1, boost::geometry::distance(p, p1)};

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  if (c2 <= c1) return {p2, boost::geometry::distance(p, p2)};

  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};
  return {projection_point, boost::geometry::distance(p, projection_point)};
}

std::optional<std::vector<std::pair<Point2d, double>>> predicted_path_to_lane_boundary_dist(
  const std::vector<Segment2d> & segments1, const std::vector<Segment2d> & segments2)
{
  if (segments1.empty() || segments2.empty()) {
    return std::nullopt;
  }

  const auto rtree_bbox_opt = build_rtree(segments2);
  if (!rtree_bbox_opt) {
    return std::nullopt;
  }
  // std::ofstream svg("/home/zulfaqar/Pictures/image.svg");  // /!\ CHANGE PATH
  // boost::geometry::svg_mapper<Point2d> mapper(svg, 400, 400);

  const auto & rtree_bbox = *rtree_bbox_opt;
  std::vector<std::pair<Point2d, double>> projections;
  projections.reserve(segments1.size());
  for (const auto & seg : segments1) {
    const auto & p1 = seg.first;
    constexpr auto num_of_boundary_segment_to_query = 10;
    std::vector<SegmentWithIdx> nearest_segments;
    nearest_segments.reserve(num_of_boundary_segment_to_query);  // ✅ Preallocate space

    rtree_bbox.query(
      bgi::nearest(p1, num_of_boundary_segment_to_query), std::back_inserter(nearest_segments));

    std::pair<Point2d, double> min_projection = {Point2d(), std::numeric_limits<double>::max()};
    for (const auto & [lane_bbox, idx] : nearest_segments) {
      const auto curr_projection = point_to_segment_signed_projection(p1, segments2[idx]);
      if (std::abs(min_projection.second) > std::abs(curr_projection.second)) {
        // mapper.add(segments2[idx]);
        // mapper.add(seg);
        // mapper.map(segments2[idx], "fill-opacity:0.3;fill:red;stroke:red;stroke-width:2");
        // mapper.map(seg, "fill-opacity:0.3;fill:blue;stroke:blue;stroke-width:2");
        min_projection = curr_projection;
      }
    }

    projections.emplace_back(min_projection);
  }
  return projections;
}
}  // namespace

namespace autoware::lane_departure_checker
{
Output LaneDepartureChecker::update(const Input & input)
{
  Output output{};

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  output.trajectory_deviation = utils::calcTrajectoryDeviation(
    *input.reference_trajectory, input.current_odom->pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold);
  output.processing_time_map["calcTrajectoryDeviation"] = stop_watch.toc(true);

  {
    constexpr double min_velocity = 0.01;
    const auto & raw_abs_velocity = std::abs(input.current_odom->twist.twist.linear.x);
    const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;

    const auto braking_distance = std::max(
      param_.min_braking_distance,
      calcBrakingDistance(abs_velocity, param_.max_deceleration, param_.delay_time));

    output.resampled_trajectory = utils::cutTrajectory(
      utils::resampleTrajectory(*input.predicted_trajectory, param_.resample_interval),
      braking_distance);
    output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);
  }
  output.vehicle_footprints = utils::createVehicleFootprints(
    input.current_odom->pose, output.resampled_trajectory, *vehicle_info_ptr_,
    param_.footprint_margin_scale);

  stop_watch.tic("find_min_dist");
  for (const auto & footprint : output.vehicle_footprints) {
    output.ego_footprint_side.right.emplace_back(
      Point2d(footprint.at(1)), Point2d(footprint.at(3)));
    output.ego_footprint_side.left.emplace_back(Point2d(footprint.at(6)), Point2d(footprint.at(4)));
  }

  if (!output.ego_footprint_side.left.empty()) {
    const auto back_point = autoware_utils::to_msg(
      output.ego_footprint_side.left.end()->first.to_3d(input.current_odom->pose.pose.position.z));
    if (
      const auto left_lane_segments_opt =
        utils::convert_to_segments(input.left_lane_boundary, back_point)) {
      output.lane_boundary.left = *left_lane_segments_opt;
    }
  }

  if (!output.ego_footprint_side.right.empty()) {
    const auto back_point = autoware_utils::to_msg(
      output.ego_footprint_side.right.end()->first.to_3d(input.current_odom->pose.pose.position.z));
    if (
      const auto right_lane_segments_opt =
        utils::convert_to_segments(input.right_lane_boundary, back_point)) {
      output.lane_boundary.right = *right_lane_segments_opt;
    }
  }

  const auto dist_ego_left_to_boundary =
    predicted_path_to_lane_boundary_dist(output.ego_footprint_side.left, output.lane_boundary.left);
  const auto dist_ego_right_to_boundary = predicted_path_to_lane_boundary_dist(
    output.ego_footprint_side.right, output.lane_boundary.right);

  fmt::print("time taken to find all min distance is {} [s]\n", stop_watch.toc("find_min_dist"));
  if (dist_ego_left_to_boundary && dist_ego_right_to_boundary) {
    const auto min_to_print =
      std::min(dist_ego_left_to_boundary->size(), dist_ego_right_to_boundary->size());

    for (size_t i = 0; i < min_to_print; ++i) {
      output.projection_points.left.emplace_back(
        output.ego_footprint_side.left[i].first, dist_ego_left_to_boundary->at(i).first);
      output.projection_points.right.emplace_back(
        output.ego_footprint_side.right[i].first, dist_ego_right_to_boundary->at(i).first);
      fmt::print(
        "at idx {} left = {} [m], right = {} [m]\n", i, dist_ego_left_to_boundary->at(i).second,
        dist_ego_right_to_boundary->at(i).second);
    }
  }

  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = utils::createVehiclePassingAreas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  const auto candidate_road_lanelets =
    utils::getCandidateLanelets(input.route_lanelets, output.vehicle_footprints);
  const auto candidate_shoulder_lanelets =
    utils::getCandidateLanelets(input.shoulder_lanelets, output.vehicle_footprints);
  output.candidate_lanelets = candidate_road_lanelets;
  output.candidate_lanelets.insert(
    output.candidate_lanelets.end(), candidate_shoulder_lanelets.begin(),
    candidate_shoulder_lanelets.end());

  output.processing_time_map["getCandidateLanelets"] = stop_watch.toc(true);

  output.will_leave_lane = willLeaveLane(output.candidate_lanelets, output.vehicle_footprints);
  output.processing_time_map["willLeaveLane"] = stop_watch.toc(true);

  output.is_out_of_lane = isOutOfLane(output.candidate_lanelets, output.vehicle_footprints.front());
  output.processing_time_map["isOutOfLane"] = stop_watch.toc(true);

  const double max_search_length_for_boundaries =
    utils::calcMaxSearchLengthForBoundaries(*input.predicted_trajectory, *vehicle_info_ptr_);
  const auto uncrossable_boundaries = extractUncrossableBoundaries(
    *input.lanelet_map, input.predicted_trajectory->points.front().pose.position,
    max_search_length_for_boundaries, input.boundary_types_to_detect);
  output.will_cross_boundary = willCrossBoundary(output.vehicle_footprints, uncrossable_boundaries);
  output.processing_time_map["willCrossBoundary"] = stop_watch.toc(true);

  return output;
}

bool LaneDepartureChecker::checkPathWillLeaveLane(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  lanelet::ConstLanelets candidate_lanelets =
    utils::getCandidateLanelets(lanelets, vehicle_footprints);
  return willLeaveLane(candidate_lanelets, vehicle_footprints);
}

bool LaneDepartureChecker::willLeaveLane(
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

std::vector<std::pair<double, lanelet::Lanelet>> LaneDepartureChecker::getLaneletsFromPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Get Footprint Hull basic polygon
  std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  LinearRing2d footprint_hull = utils::createHullFromFootprints(vehicle_footprints);

  lanelet::BasicPolygon2d footprint_hull_basic_polygon = toBasicPolygon2D(footprint_hull);

  // Find all lanelets that intersect the footprint hull
  return lanelet::geometry::findWithin2d(
    lanelet_map_ptr->laneletLayer, footprint_hull_basic_polygon, 0.0);
}

std::optional<autoware_utils::Polygon2d> LaneDepartureChecker::getFusedLaneletPolygonForPath(
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

bool LaneDepartureChecker::updateFusedLaneletPolygonForPath(
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

bool LaneDepartureChecker::checkPathWillLeaveLane(
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

bool LaneDepartureChecker::checkPathWillLeaveLane(
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

PathWithLaneId LaneDepartureChecker::cropPointsOutsideOfLanes(
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

bool LaneDepartureChecker::isOutOfLane(
  const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : vehicle_footprint) {
    if (!isInAnyLane(candidate_lanelets, point)) {
      return true;
    }
  }

  return false;
}

SegmentRtree LaneDepartureChecker::extractUncrossableBoundaries(
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

bool LaneDepartureChecker::willCrossBoundary(
  const std::vector<LinearRing2d> & vehicle_footprints,
  const SegmentRtree & uncrossable_segments) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & footprint : vehicle_footprints) {
    std::vector<Segment2d> intersection_result;
    uncrossable_segments.query(
      boost::geometry::index::intersects(footprint), std::back_inserter(intersection_result));
    if (!intersection_result.empty()) {
      return true;
    }
  }
  return false;
}

lanelet::BasicPolygon2d LaneDepartureChecker::toBasicPolygon2D(
  const LinearRing2d & footprint_hull) const
{
  lanelet::BasicPolygon2d basic_polygon;
  for (const auto & point : footprint_hull) {
    Eigen::Vector2d p(point.x(), point.y());
    basic_polygon.push_back(p);
  }
  return basic_polygon;
}

autoware_utils::Polygon2d LaneDepartureChecker::toPolygon2D(
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

}  // namespace autoware::lane_departure_checker
