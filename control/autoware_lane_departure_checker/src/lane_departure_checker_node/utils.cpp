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

#include "autoware/lane_departure_checker/utils.hpp"

#include "autoware/lane_departure_checker/parameters.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <vector>

namespace
{
struct FootprintMargin
{
  double lon;
  double lat;
};

FootprintMargin calcFootprintMargin(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto Cov_in_map = covariance.covariance;
  Eigen::Matrix2d Cov_xy_map;
  Cov_xy_map << Cov_in_map[0 * 6 + 0], Cov_in_map[0 * 6 + 1], Cov_in_map[1 * 6 + 0],
    Cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // To get a position in a transformed coordinate, rotate the inverse direction
  Eigen::Matrix2d R_map2vehicle;
  R_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);
  // Rotate covariance E((X, Y)^t*(X, Y)) = E(R*(x,y)*(x,y)^t*R^t)
  // when Rotate point (X, Y)^t= R*(x, y)^t.
  const Eigen::Matrix2d Cov_xy_vehicle = R_map2vehicle * Cov_xy_map * R_map2vehicle.transpose();

  // The longitudinal/lateral length is represented
  // in Cov_xy_vehicle(0,0), Cov_xy_vehicle(1,1) respectively.
  return FootprintMargin{Cov_xy_vehicle(0, 0) * scale, Cov_xy_vehicle(1, 1) * scale};
}
}  // namespace

namespace autoware::lane_departure_checker::utils
{
using autoware_utils::Segment2d;
using lane_departure_checker::Projection;
TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length)
{
  if (trajectory.empty()) {
    return {};
  }

  TrajectoryPoints cut;

  double total_length = 0.0;
  auto last_point = autoware_utils::from_msg(trajectory.front().pose.position);
  auto end_it = std::next(trajectory.cbegin());
  for (; end_it != trajectory.cend(); ++end_it) {
    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0) {
      break;
    }

    const auto & new_pose = end_it->pose;
    const auto new_point = autoware_utils::from_msg(new_pose.position);
    const auto points_distance = boost::geometry::distance(last_point.to_2d(), new_point.to_2d());

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated =
        last_point + remain_distance * (new_point - last_point).normalized();

      TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = new_pose.orientation;

      cut.push_back(p);
      break;
    }

    total_length += points_distance;
    last_point = new_point;
  }
  cut.insert(cut.begin(), trajectory.begin(), end_it);

  return cut;
}

TrajectoryPoints resampleTrajectory(const Trajectory & trajectory, const double interval)
{
  if (trajectory.points.size() < 2) {
    return trajectory.points;
  }

  TrajectoryPoints resampled;

  resampled.push_back(trajectory.points.front());
  auto prev_point = autoware_utils::from_msg(trajectory.points.front().pose.position);
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & traj_point = trajectory.points.at(i);

    const auto next_point = autoware_utils::from_msg(traj_point.pose.position);

    if (boost::geometry::distance(prev_point.to_2d(), next_point.to_2d()) >= interval) {
      resampled.push_back(traj_point);
      prev_point = next_point;
    }
  }
  resampled.push_back(trajectory.points.back());

  return resampled;
}

std::vector<std::pair<LinearRing2d, Pose>> createVehicleFootprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_margin_scale)
{
  // Calculate longitudinal and lateral margin based on covariance
  const auto margin = calcFootprintMargin(covariance, footprint_margin_scale);

  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat, margin.lon);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<std::pair<LinearRing2d, Pose>> vehicle_footprints;
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const auto & p) -> decltype(vehicle_footprints) {
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;
      return {transform_vector(local_vehicle_footprint, pose2transform(p.pose)), p.pose};
    });

  return vehicle_footprints;
}

std::vector<LinearRing2d> createVehicleFootprints(
  const PathWithLaneId & path, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_extra_margin)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(footprint_extra_margin);

  // Create vehicle footprint on each Path point
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : path.points) {
    vehicle_footprints.push_back(autoware_utils::transform_vector(
      local_vehicle_footprint, autoware_utils::pose2transform(p.point.pose)));
  }

  return vehicle_footprints;
}

lanelet::ConstLanelets getCandidateLanelets(
  const lanelet::ConstLanelets & route_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  lanelet::ConstLanelets candidate_lanelets;

  // Find lanes within the convex hull of footprints
  const auto footprint_hull = createHullFromFootprints(vehicle_footprints);

  for (const auto & route_lanelet : route_lanelets) {
    const auto poly = route_lanelet.polygon2d().basicPolygon();
    if (!boost::geometry::disjoint(poly, footprint_hull)) {
      candidate_lanelets.push_back(route_lanelet);
    }
  }

  return candidate_lanelets;
}

LinearRing2d createHullFromFootprints(const std::vector<LinearRing2d> & footprints)
{
  MultiPoint2d combined;
  for (const auto & footprint : footprints) {
    for (const auto & p : footprint) {
      combined.push_back(p);
    }
  }

  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);

  return hull;
}

std::vector<LinearRing2d> createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  if (vehicle_footprints.empty()) {
    return {};
  }

  if (vehicle_footprints.size() == 1) {
    return {vehicle_footprints.front()};
  }

  std::vector<LinearRing2d> areas;
  areas.reserve(vehicle_footprints.size() - 1);

  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(createHullFromFootprints({footprint1, footprint2}));
  }

  return areas;
}

PoseDeviation calcTrajectoryDeviation(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points, pose, dist_threshold, yaw_threshold);
  return autoware_utils::calc_pose_deviation(trajectory.points.at(nearest_idx).pose, pose);
}

double calcMaxSearchLengthForBoundaries(
  const Trajectory & trajectory, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const double max_ego_lon_length = std::max(
    std::abs(vehicle_info.max_longitudinal_offset_m),
    std::abs(vehicle_info.min_longitudinal_offset_m));
  const double max_ego_lat_length = std::max(
    std::abs(vehicle_info.max_lateral_offset_m), std::abs(vehicle_info.min_lateral_offset_m));
  const double max_ego_search_length = std::hypot(max_ego_lon_length, max_ego_lat_length);
  return autoware::motion_utils::calcArcLength(trajectory.points) + max_ego_search_length;
}

lanelet::BoundingBox2d create_bbox(
  const geometry_msgs::msg::Point & point, const double search_distance)
{
  constexpr auto eps{1e-3};
  if (search_distance < eps) {
    return {
      lanelet::BasicPoint2d{point.x - eps, point.y - eps},
      lanelet::BasicPoint2d{point.x + eps, point.y + eps}};
  }
  return {
    lanelet::BasicPoint2d{point.x - search_distance, point.y - search_distance},
    lanelet::BasicPoint2d{point.x + search_distance, point.y + search_distance}};
}

std::vector<lanelet::PrimitiveLayer<lanelet::LineString3d>::ConstPrimitiveT> get_nearby_linestrings(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Point & point,
  const double search_distance)
{
  const auto search_bbox = create_bbox(point, search_distance);
  return lanelet_map.lineStringLayer.search(search_bbox);
}

std::unordered_map<lanelet::Id, lanelet::BasicLineString3d> get_nearby_uncrossable_boundaries(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Point & point,
  const double search_distance, const std::vector<std::string> & boundary_types_to_detect)
{
  const auto nearby_linestrings = get_nearby_linestrings(lanelet_map, point, search_distance);
  std::unordered_map<lanelet::Id, lanelet::BasicLineString3d> uncrossable_boundaries;

  const auto is_uncrossable_type = [&boundary_types_to_detect](const auto & ls) {
    constexpr auto no_type = "";
    const auto & types = boundary_types_to_detect;
    const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
    return (type != no_type && std::find(types.begin(), types.end(), type) != types.end());
  };

  uncrossable_boundaries.reserve(nearby_linestrings.size());
  for (const auto & ls : nearby_linestrings) {
    if (!is_uncrossable_type(ls)) {
      continue;
    }
    uncrossable_boundaries.insert(std::make_pair(ls.id(), ls.basicLineString()));
  }
  return uncrossable_boundaries;
}

std::optional<Projection> point_to_segment_signed_projection(
  const Point2d & p, const Segment2d & segment, const bool swap_points = false)
{
  const auto & p1 = segment.first;
  const auto & p2 = segment.second;

  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto result = [&swap_points](
                        const Point2d & orig, const Point2d & proj, const double dist) {
    return swap_points ? Projection{proj, orig, dist} : Projection{orig, proj, dist};
  };

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 < 0.0) return std::nullopt;
  if (c1 == 0.0) return result(p, p1, boost::geometry::distance(p, p1));

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  if (c1 > c2) return std::nullopt;

  if (c1 == c2) return result(p, p2, boost::geometry::distance(p, p2));

  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};
  return result(p, projection_point, boost::geometry::distance(p, projection_point));
}

std::optional<Projection> segment_to_segment_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg)
{
  std::vector<Projection> projections;

  if (const auto projection_opt = point_to_segment_signed_projection(ego_seg.first, lane_seg)) {
    projections.push_back(*projection_opt);
  }

  if (const auto projection_opt = point_to_segment_signed_projection(ego_seg.second, lane_seg)) {
    projections.push_back(*projection_opt);
  }

  constexpr bool swap_result = true;
  if (
    const auto projection_opt =
      point_to_segment_signed_projection(lane_seg.first, ego_seg, swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (
    const auto projection_opt =
      point_to_segment_signed_projection(lane_seg.second, ego_seg, swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (projections.empty()) return std::nullopt;
  if (projections.size() == 1) return projections.front();

  const auto min_elem = std::min_element(
    projections.begin(), projections.end(), [](const Projection & proj1, const Projection & proj2) {
      return std::abs(proj1.distance) < std::abs(proj2.distance);
    });

  return *min_elem;
}

SegmentRtree build_rtree(
  const std::unordered_map<lanelet::Id, lanelet::BasicLineString3d> & uncrossable_boundaries)
{
  std::vector<SegmentWithIdx> boxes_with_idx;
  for (size_t i = 0; i < uncrossable_boundaries.size(); ++i) {
    Segment2d segment2d;
    bg::envelope(uncrossable_boundaries[i], segment2d);
    boxes_with_idx.emplace_back(segment2d, i);
  }
  return SegmentRtree(boxes_with_idx.begin(), boxes_with_idx.end());
}

std::optional<std::vector<Projection>> predicted_path_to_lane_boundary_dist(
  const std::vector<Segment2d> & ego_side_segments,
  const std::vector<Segment2d> & lane_side_segments)
{
  if (ego_side_segments.empty() || lane_side_segments.empty()) {
    return std::nullopt;
  }

  const auto rtree_bbox_opt = build_rtree(lane_side_segments);
  if (!rtree_bbox_opt) {
    return std::nullopt;
  }

  const auto & rtree_bbox = *rtree_bbox_opt;
  std::vector<Projection> projections;
  projections.reserve(ego_side_segments.size());
  for (const auto & seg : ego_side_segments) {
    constexpr auto num_of_boundary_segment_to_query = 1;
    std::vector<SegmentWithIdx> nearest_segments;
    nearest_segments.reserve(num_of_boundary_segment_to_query);  // ✅ Preallocate space
    rtree_bbox.query(
      bgi::nearest(seg, num_of_boundary_segment_to_query), std::back_inserter(nearest_segments));

    std::optional<Projection> min_projection_opt;
    geometry_msgs::msg::Point prev_intersect;
    for (const auto & [lane_bbox, idx] : nearest_segments) {
      const auto intersect = autoware_utils::intersect(
        autoware_utils::to_msg(seg.first.to_3d()), autoware_utils::to_msg(seg.second.to_3d()),
        autoware_utils::to_msg(lane_side_segments[idx].first.to_3d()),
        autoware_utils::to_msg(lane_side_segments[idx].second.to_3d()));
      if (intersect) {
        prev_intersect = *intersect;
        break;
      }
      const auto curr_projection =
        segment_to_segment_nearest_projection(seg, lane_side_segments[idx]);
      if (
        !min_projection_opt ||
        std::abs(min_projection_opt->distance) > std::abs(curr_projection.distance)) {
        min_projection_opt = curr_projection;
      }
    }

    if (min_projection_opt) {
      projections.emplace_back(*min_projection_opt);
    } else {
      std::optional<Projection> min_lane_projection_opt;
      std::vector<SegmentWithIdx> nearest_route_segments;
      nearest_route_segments.reserve(num_of_boundary_segment_to_query);  // ✅ Preallocate space
      rtree_bbox.query(
        bgi::nearest(seg, num_of_boundary_segment_to_query),
        std::back_inserter(nearest_route_segments));
      for (const auto & [lane_bbox, idx] : nearest_route_segments) {
        const auto intersect_with_route = autoware_utils::intersect(
          autoware_utils::to_msg(seg.first.to_3d()), autoware_utils::to_msg(seg.second.to_3d()),
          autoware_utils::to_msg(lane_side_segments[idx].first.to_3d()),
          autoware_utils::to_msg(lane_side_segments[idx].second.to_3d()));
        if (intersect_with_route) {
          if (
            autoware_utils::calc_distance2d(
              autoware_utils::to_msg(seg.second.to_3d()), *intersect_with_route) >
            autoware_utils::calc_distance2d(
              autoware_utils::to_msg(seg.second.to_3d()), prev_intersect)) {
            prev_intersect = *intersect_with_route;
          }
        }
        const auto curr_projection =
          segment_to_segment_nearest_projection(seg, lane_side_segments[idx]);
        if (
          !min_lane_projection_opt ||
          std::abs(min_projection_opt->distance) > std::abs(curr_projection.distance)) {
          min_projection_opt = curr_projection;
        }
      }
    }
  }
  return projections;
}
}  // namespace autoware::lane_departure_checker::utils
