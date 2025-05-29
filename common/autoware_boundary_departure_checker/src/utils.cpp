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

#include "autoware/boundary_departure_checker/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <range/v3/view.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace
{
bool is_uncrossable_type(
  std::vector<std::string> boundary_types_to_detect, const lanelet::ConstLineString3d & ls)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  return (
    type != no_type &&
    std::find(boundary_types_to_detect.begin(), boundary_types_to_detect.end(), type) !=
      boundary_types_to_detect.end());
};
}  // namespace

namespace autoware::boundary_departure_checker::utils
{
using autoware_utils::Segment2d;

Point2d to_point2d(const Eigen::Matrix<double, 3, 1> & ll_pt)
{
  return {ll_pt.x(), ll_pt.y()};
}

Segment2d to_segment2d(
  const Eigen::Matrix<double, 3, 1> & ll_pt1, const Eigen::Matrix<double, 3, 1> & ll_pt2)
{
  return {to_point2d(ll_pt1), to_point2d(ll_pt2)};
}

std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const FootprintMargin & margin)
{
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat_m, margin.lon_m);

  std::vector<LinearRing2d> vehicle_footprints;
  vehicle_footprints.reserve(trajectory.size());
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const auto & p) -> LinearRing2d {
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;
      return transform_vector(local_vehicle_footprint, pose2transform(p.pose));
    });

  return vehicle_footprints;
}

std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const SteeringReport & current_steering)
{
  constexpr auto steering_rate_gain = 1.0;
  constexpr auto steering_rate_rad_per_s = 0.25;

  std::vector<LinearRing2d> vehicle_footprints;
  vehicle_footprints.reserve(trajectory.size());
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const TrajectoryPoint & p) -> LinearRing2d {
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;
      const double raw_angle_rad =
        current_steering.steering_tire_angle +
        (steering_rate_rad_per_s * rclcpp::Duration(p.time_from_start).seconds());

      constexpr auto min_angle = autoware_utils::deg2rad(-89);
      constexpr auto max_angle = autoware_utils::deg2rad(89);
      const double clamped_angle_rad = std::clamp(raw_angle_rad, min_angle, max_angle);

      const auto local_vehicle_footprint = vehicle_info.createFootprint(
        std::max(std::tan(clamped_angle_rad) * steering_rate_gain, 0.0), 0.0, 0.0, 0.0, 0.0, true);
      return transform_vector(local_vehicle_footprint, pose2transform(p.pose));
    });

  if (vehicle_footprints.empty() || vehicle_footprints.front().size() < 6) {
    return vehicle_footprints;
  }

  LinearRing2d footprint{vehicle_footprints.back()[6]};

  for (const auto & fp : vehicle_footprints | ranges::views::reverse) {
    footprint.push_back(fp[1]);
  }
  footprint.push_back(vehicle_footprints.front()[3]);
  footprint.push_back(vehicle_footprints.front()[4]);

  for (const auto & fp : vehicle_footprints) {
    footprint.push_back(fp[6]);
  }

  return {footprint};
}

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
  const auto margin =
    utils::calc_extra_margin_from_pose_covariance(covariance, footprint_margin_scale);

  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat_m, margin.lon_m);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<std::pair<LinearRing2d, Pose>> vehicle_footprints;
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const auto & p) -> std::pair<LinearRing2d, Pose> {
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

std::optional<Projection> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment, const bool swap_points = false)
{
  const auto & p1 = segment.first;
  const auto & p2 = segment.second;

  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto result = [&swap_points](const Point2d & orig, const Point2d & proj) {
    return swap_points ? Projection{proj, orig, boost::geometry::distance(proj, orig)}
                       : Projection{orig, proj, boost::geometry::distance(orig, proj)};
  };

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 < 0.0) return std::nullopt;
  if (c1 == 0.0) return result(p, p1);

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  if (c1 > c2) return std::nullopt;

  if (c1 == c2) return result(p, p2);

  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};

  return result(p, projection_point);
}

std::optional<Projection> segment_to_segment_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg)
{
  const auto & [ego_f, ego_b] = ego_seg;
  const auto & [lane_pt1, lane_pt2] = lane_seg;
  if (
    const auto is_intersecting = autoware_utils::intersect(
      autoware_utils::to_msg(ego_f.to_3d()), autoware_utils::to_msg(ego_b.to_3d()),
      autoware_utils::to_msg(lane_pt1.to_3d()), autoware_utils::to_msg(lane_pt2.to_3d()))) {
    Point2d point(is_intersecting->x, is_intersecting->y);
    return Projection{point, point, 0.0};
  }

  std::vector<Projection> projections;
  projections.reserve(4);
  constexpr bool swap_result = true;
  if (const auto projection_opt = point_to_segment_projection(ego_f, lane_seg, swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (const auto projection_opt = point_to_segment_projection(ego_b, lane_seg, swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (const auto projection_opt = point_to_segment_projection(lane_pt1, ego_seg, !swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (const auto projection_opt = point_to_segment_projection(lane_pt2, ego_seg, !swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (projections.empty()) return std::nullopt;
  if (projections.size() == 1) return projections.front();

  const auto min_elem = std::min_element(
    projections.begin(), projections.end(), [](const Projection & proj1, const Projection & proj2) {
      return std::abs(proj1.dist) < std::abs(proj2.dist);
    });

  return *min_elem;
}

std::vector<SegmentWithIdx> create_local_segments(const lanelet::ConstLineString3d & linestring)
{
  std::vector<SegmentWithIdx> local_segments;
  local_segments.reserve(linestring.size());
  const auto basic_ls = linestring.basicLineString();
  for (size_t i = 0; i + 1 < basic_ls.size(); ++i) {
    const auto segment = to_segment2d(basic_ls.at(i), basic_ls.at(i + 1));
    local_segments.emplace_back(bg::return_envelope<Segment2d>(segment), linestring.id(), i, i + 1);
  }
  return local_segments;
}

UncrossableBoundRTree build_uncrossable_boundaries_rtree(
  const lanelet::LineStringLayer & linestring_layer,
  const std::vector<std::string> & boundary_types_to_detect)
{
  std::vector<SegmentWithIdx> segments;
  for (const auto & linestring : linestring_layer) {
    if (!is_uncrossable_type(boundary_types_to_detect, linestring)) {
      continue;
    }

    auto local_segments = create_local_segments(linestring);
    std::move(local_segments.begin(), local_segments.end(), std::back_inserter(segments));
  }

  return {segments.begin(), segments.end()};
}

UncrossableBoundRTree build_uncrossable_boundaries_rtree(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect)
{
  return build_uncrossable_boundaries_rtree(lanelet_map.lineStringLayer, boundary_types_to_detect);
}

BoundarySideWithIdx get_boundary_segments_from_side(
  const UncrossableBoundRTree & rtree, const lanelet::LineStringLayer & linestring_layer,
  const EgoSides & ego_sides_from_footprints, const int max_lat_query_num)
{
  const auto closest_segment =
    [&](const auto & ego_seg, const auto & compare_seg, auto & output_side) {
      std::vector<SegmentWithIdx> nearest_raw;

      const lanelet::BasicPoint2d ego_start{ego_seg.first.x(), ego_seg.first.y()};
      rtree.query(bgi::nearest(ego_start, max_lat_query_num), std::back_inserter(nearest_raw));

      for (const auto & nearest : nearest_raw) {
        const auto ll_id = std::get<1>(nearest);
        const auto basic_ls = linestring_layer.get(ll_id).basicLineString();

        const auto idx_curr = std::get<2>(nearest);
        const auto idx_next = std::get<3>(nearest);
        const auto seg = to_segment2d(basic_ls.at(idx_curr), basic_ls.at(idx_next));

        const auto dist_from_curr_side = bg::comparable_distance(ego_seg, seg);
        const auto dist_from_compare_side = bg::comparable_distance(compare_seg, seg);

        if (dist_from_compare_side < dist_from_curr_side) {
          continue;
        }

        const auto found = [&](const SegmentWithIdx & output_seg) {
          const auto & [out_seg, out_ll_id, out_ls_idx_curr, out_ls_idx_next] = output_seg;
          return out_ll_id == ll_id && out_ls_idx_curr == idx_curr;
        };

        if (!std::any_of(output_side.begin(), output_side.end(), found)) {
          output_side.push_back(std::make_tuple(seg, ll_id, idx_curr, idx_next));
        }
      }
    };

  BoundarySideWithIdx output;
  for (const auto & fp : ego_sides_from_footprints) {
    closest_segment(fp.left, fp.right, output.left);   // Left side
    closest_segment(fp.right, fp.left, output.right);  // Right side
  }

  return output;
}

SideToBoundPojections get_closest_boundary_segments_from_side(
  const BoundarySideWithIdx & boundaries, const EgoSides & ego_sides_from_footprints)
{
  const auto closest_segment =
    [&](
      const auto & ego_seg, const auto curr_fp_idx,
      const auto & boundary_segments) -> std::optional<ProjectionWithSegment> {
    std::optional<ProjectionWithSegment> closest_proj;
    for (const auto & [seg, ll_id, idx_curr, idx_next] : boundary_segments) {
      if (const auto proj_opt = segment_to_segment_nearest_projection(ego_seg, seg)) {
        if (!closest_proj || proj_opt->dist < closest_proj->projection.dist) {
          closest_proj = ProjectionWithSegment{*proj_opt, seg, curr_fp_idx};
        }
      }
    }

    if (closest_proj) {
      return *closest_proj;
    }
    return std::nullopt;
  };

  SideToBoundPojections side;
  side.left.reserve(ego_sides_from_footprints.size());
  side.right.reserve(ego_sides_from_footprints.size());

  for (size_t i = 0; i < ego_sides_from_footprints.size(); ++i) {
    const auto & fp = ego_sides_from_footprints[i];
    if (const auto left_segment_opt = closest_segment(fp.left, i, boundaries.left)) {
      side.left.push_back(*left_segment_opt);
    }

    if (const auto right_segment_opt = closest_segment(fp.right, i, boundaries.right)) {
      side.right.push_back(*right_segment_opt);
    }
  }

  return side;
}

lanelet::BasicPolygon2d toBasicPolygon2D(const LinearRing2d & footprint_hull)
{
  lanelet::BasicPolygon2d basic_polygon;
  basic_polygon.reserve(footprint_hull.size());
  for (const auto & point : footprint_hull) {
    Eigen::Vector2d p(point.x(), point.y());
    basic_polygon.push_back(p);
  }
  return basic_polygon;
}

Side<Segment2d> get_footprint_sides(
  const LinearRing2d & footprint, const bool use_center_right, const bool use_center_left)
{
  const auto center_right = use_center_right ? 2 : 3;
  const auto center_left = use_center_left ? 5 : 4;

  const auto & right_front = footprint[1];
  const auto & right_back = footprint[center_right];

  const auto & left_front = footprint[6];
  const auto & left_back = footprint[center_left];

  Side<Segment2d> side;
  side.right = {Point2d(right_front.x(), right_front.y()), Point2d(right_back.x(), right_back.y())};
  side.left = {Point2d(left_front.x(), left_front.y()), Point2d(left_back.x(), left_back.y())};

  return side;
}

LineString2d to_linestring_2d(const Segment2d & segment)
{
  const auto & [fr, bk] = segment;
  return {fr, bk};
}

double cross_2d(const Point2d & point, const Segment2d & seg)
{
  const auto & [p1, p2] = seg;
  return (p2.x() - p1.x()) * (point.y() - p1.y()) - (p2.y() - p1.y()) * (point.x() - p1.x());
}

bool is_point_left_of_line(const Point2d & point, const Segment2d & seg)
{
  return cross_2d(point, seg) > 0.0;
}

std::vector<lanelet::ConstLineString3d> get_linestrings_near_footprint(
  const lanelet::LineStringLayer & linestring_layer, const Pose & ego_pose,
  const double search_distance, const std::vector<std::string> uncrossable_boundary_types)
{
  const auto bbox = lanelet::BoundingBox2d(
    lanelet::BasicPoint2d{
      ego_pose.position.x - search_distance, ego_pose.position.y - search_distance},
    lanelet::BasicPoint2d{
      ego_pose.position.x + search_distance, ego_pose.position.y + search_distance});

  auto nearby_linestrings = linestring_layer.search(bbox);

  const auto remove_itr = std::remove_if(
    nearby_linestrings.begin(), nearby_linestrings.end(),
    [&](const auto & ls) { return !is_uncrossable_type(uncrossable_boundary_types, ls); });

  nearby_linestrings.erase(remove_itr, nearby_linestrings.end());

  return nearby_linestrings;
}

FootprintMargin calc_extra_margin_from_pose_covariance(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto cov_in_map = covariance.covariance;
  Eigen::Matrix2d cov_xy_map;
  cov_xy_map << cov_in_map[0 * 6 + 0], cov_in_map[0 * 6 + 1], cov_in_map[1 * 6 + 0],
    cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // To get a position in a transformed coordinate, rotate the inverse direction
  Eigen::Matrix2d r_map2vehicle;
  r_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);
  // Rotate covariance E((X, Y)^t*(X, Y)) = E(R*(x,y)*(x,y)^t*R^t)
  // when Rotate point (X, Y)^t= R*(x, y)^t.
  const Eigen::Matrix2d cov_xy_vehicle = r_map2vehicle * cov_xy_map * r_map2vehicle.transpose();

  // The longitudinal/lateral length is represented
  // in cov_xy_vehicle(0,0), cov_xy_vehicle(1,1) respectively.
  return FootprintMargin{cov_xy_vehicle(0, 0) * scale, cov_xy_vehicle(1, 1) * scale};
}

tl::expected<std::vector<PoseWithDist>, std::string> get_poses_with_dist_on_trajectory(
  const TrajectoryPoints & ego_pred_traj,
  const trajectory::Trajectory<TrajectoryPoint> & aw_raw_traj)
{
  if (ego_pred_traj.empty()) {
    return tl::make_unexpected("Ego predicted trajectory is empty");
  }

  std::vector<PoseWithDist> poses_with_dist;
  const auto underlying_bases = aw_raw_traj.get_underlying_bases();
  poses_with_dist.reserve(underlying_bases.size());

  for (const auto & pred_traj_pt : ego_pred_traj) {
    const auto dist_from_start = trajectory::closest(aw_raw_traj, pred_traj_pt);
    poses_with_dist.emplace_back(pred_traj_pt.pose, dist_from_start);
  }

  return poses_with_dist;
}

EgoSide get_ego_side_from_footprint(
  const Footprint & fp, const PoseWithDist & pose_with_dist, const bool use_center_right,
  const bool use_center_left)
{
  const auto & [pose, d] = pose_with_dist;
  auto fp_side = get_footprint_sides(fp, use_center_right, use_center_left);
  EgoSide ego_side;
  ego_side.left = std::move(fp_side.left);
  ego_side.right = std::move(fp_side.right);
  ego_side.dist_from_start = d;
  ego_side.pose = pose;
  return ego_side;
}

tl::expected<EgoSides, std::string> get_ego_sides_from_footprints(
  const Footprints & footprints, const std::vector<PoseWithDist> & poses_on_traj)
{
  if (footprints.empty() || poses_on_traj.empty()) {
    return tl::make_unexpected("Footprint or trajectory is empty");
  }

  if (footprints.size() != poses_on_traj.size()) {
    return tl::make_unexpected("footprint size does not match trajectory size");
  }

  EgoSides footprints_sides;
  footprints_sides.reserve(footprints.size());
  constexpr bool use_center_right = true;
  constexpr bool use_center_left = true;

  for (const auto & [fp, pose_with_dist] : ranges::views::zip(footprints, poses_on_traj)) {
    auto ego_side =
      get_ego_side_from_footprint(fp, pose_with_dist, use_center_right, use_center_left);
    footprints_sides.push_back(ego_side);
  }

  return footprints_sides;
}

}  // namespace autoware::boundary_departure_checker::utils
