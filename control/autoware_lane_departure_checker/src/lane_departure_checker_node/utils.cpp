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

#include <autoware/lane_departure_checker/kdtree.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>

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
using autoware_utils::Point2d;

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

std::vector<LinearRing2d> createVehicleFootprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_margin_scale)
{
  // Calculate longitudinal and lateral margin based on covariance
  const auto margin = calcFootprintMargin(covariance, footprint_margin_scale);

  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat, margin.lon);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory) {
    vehicle_footprints.push_back(autoware_utils::transform_vector(
      local_vehicle_footprint, autoware_utils::pose2transform(p.pose)));
  }

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

std::optional<std::vector<Segment2d>> convert_to_segments(
  const std::vector<Point> & boundary_points, const Point & ego_point, const double search_length)
{
  if (boundary_points.size() < 2) {
    return std::nullopt;
  }

  std::vector<Segment2d> segments;
  segments.reserve(boundary_points.size() - 1);
  for (size_t i = 0; i < boundary_points.size() - 1; ++i) {
    if (autoware_utils::calc_distance2d(boundary_points[i], ego_point) > search_length) {
      continue;
    }
    const auto & curr = boundary_points[i];
    const auto & next = boundary_points[i + 1];
    segments.emplace_back(Point2d(curr.x, curr.y), Point2d(next.x, next.y));
  }
  return segments;
}

SegmentNodeRtreePair get_route_lanelet_bound_segment(const lanelet::ConstLanelets & route_lanelets)
{
  const auto to_segment = [](const auto & curr, const auto & next) -> Segment2d {
    const Point2d p1 = {curr.x(), curr.y()};
    const Point2d p2 = {next.x(), next.y()};
    return {p1, p2};
  };

  SegmentNodePair segments;
  for (const auto & ll : route_lanelets) {
    const auto & left = ll.leftBound3d().basicLineString();
    for (size_t i = 0; i < left.size() - 1; ++i) {
      const auto segment = to_segment(left[i], left[i + 1]);
      segments.left.emplace_back(bg::return_envelope<Segment2d>(segment), ll.id());
    }

    const auto & right = ll.rightBound3d().basicLineString();
    for (size_t i = 0; i < right.size() - 1; ++i) {
      const auto segment = to_segment(right[i], right[i + 1]);
      segments.right.emplace_back(bg::return_envelope<Segment2d>(segment), ll.id());
    }
  }
  SegmentNodeRtreePair rtree;
  rtree.left = {segments.left.begin(), segments.left.end()};
  rtree.right = {segments.right.begin(), segments.right.end()};
  return rtree;
}

SegmentNodeRtreePair extract_uncrossable_boundaries(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect)
{
  const auto has_types =
    [](const lanelet::ConstLineString3d & ls, const std::vector<std::string> & types) {
      constexpr auto no_type = "";
      const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
      return (type != no_type && std::find(types.begin(), types.end(), type) != types.end());
    };

  const auto to_segment = [](const auto & curr, const auto & next) -> Segment2d {
    const Point2d p1 = {curr.x(), curr.y()};
    const Point2d p2 = {next.x(), next.y()};
    return {p1, p2};
  };

  SegmentNodePair segments;
  for (const auto & ll : lanelet_map.laneletLayer) {
    if (has_types(ll.leftBound3d(), boundary_types_to_detect)) {
      const auto & left = ll.leftBound3d().basicLineString();
      segments.left.reserve(segments.left.size() + left.size());
      for (size_t i = 0; i < left.size() - 1; ++i) {
        const auto segment = to_segment(left[i], left[i + 1]);
        segments.left.emplace_back(bg::return_envelope<Segment2d>(segment), ll.id());
      }
    }

    if (has_types(ll.rightBound3d(), boundary_types_to_detect)) {
      const auto & right = ll.rightBound3d().basicLineString();
      segments.right.reserve(segments.right.size() + right.size());
      for (size_t i = 0; i < right.size() - 1; ++i) {
        const auto segment = to_segment(right[i], right[i + 1]);
        segments.right.emplace_back(bg::return_envelope<Segment2d>(segment), ll.id());
      }
    }
  }

  SegmentNodeRtreePair rtree;
  rtree.left = {segments.left.begin(), segments.left.end()};
  rtree.right = {segments.right.begin(), segments.right.end()};
  return rtree;
}

// --- Helper Functions ---
// RemovePoint: removes any point from dataset that is “equal” to currentPoint.
void RemovePoint(std::vector<Point2d>& dataset, const Point2d& currentPoint)
{
  constexpr double threshold = 1e-3;
  dataset.erase(
    std::remove_if(
      dataset.begin(), dataset.end(),
      [&currentPoint](const Point2d& p) {
        return (std::abs(p.x() - currentPoint.x()) < threshold) &&
               (std::abs(p.y() - currentPoint.y()) < threshold);
      }),
    dataset.end());
}

double normalizeAngle(double angle)
{
  const double TWO_PI = 2.0 * M_PI;
  while (angle < 0) angle += TWO_PI;
  while (angle >= TWO_PI) angle -= TWO_PI;
  return angle;
}

double computeCandidateAngle(const Point2d& current, const Point2d& candidate)
{
  return std::atan2(candidate.y() - current.y(), candidate.x() - current.x());
}

// Turning angle = normalized(prevAngle - candidateAngle)
double computeTurningAngle(double prevAngle, const Point2d& current, const Point2d& candidate)
{
  double candidateAngle = computeCandidateAngle(current, candidate);
  double diff = prevAngle - candidateAngle;
  return normalizeAngle(diff);
}

// SortByAngle: returns listOfPoints sorted in descending order of turning angle
// (largest right-hand turn first). The first element is the best candidate.
std::vector<Point2d> SortByAngle(
  const std::vector<Point2d> & listOfPoints, const Point2d & current, double prevAngle)
{
  std::vector<Point2d> sorted = listOfPoints;

  std::sort(
    sorted.begin(), sorted.end(), [current, prevAngle](const Point2d & a, const Point2d & b) {
      // Filter out self-points or degenerate values
      if (a.x() == current.x() && a.y() == current.y()) return false;
      if (b.x() == current.x() && b.y() == current.y()) return true;

      const double angleA = computeTurningAngle(prevAngle, current, a);
      const double angleB = computeTurningAngle(prevAngle, current, b);

      if (!std::isfinite(angleA)) return false;
      if (!std::isfinite(angleB)) return true;

      return angleA > angleB;  // descending order
    });

  return sorted;
}

// allPointsInsidePolygon: returns true if every point in dataset is within the polygon defined by hull.
bool allPointsInsidePolygon(const std::vector<Point2d>& dataset, const std::vector<Point2d>& hull)
{
    typedef bg::model::polygon<Point2d> Polygon;
    Polygon poly;
    for (const auto &p : hull)
    {
        bg::append(poly.outer(), p);
    }
    if (!hull.empty() &&
        (std::abs(hull.front().x() - hull.back().x()) > 1e-9 ||
         std::abs(hull.front().y() - hull.back().y()) > 1e-9))
    {
        bg::append(poly.outer(), hull.front());
    }
    bg::correct(poly);
    for (const auto &pt : dataset)
    {
        if (!bg::within(pt, poly))
        {
            return false;
        }
    }
    return true;
}

// --- Improved concave_hull function ---
std::vector<Point2d> concave_hull(
  const std::vector<Point2d>& point_list, const size_t num_of_neighbor)
{
  auto dataset = point_list;
  if (dataset.size() <= 3) {
    return dataset;
  }

  // Sort by y then by x to help in duplicate removal and selecting the lowest point.
  std::sort(dataset.begin(), dataset.end(), [](const Point2d& a, const Point2d& b) {
    if (a.y() != b.y()) return a.y() < b.y();
    return a.x() < b.x();
  });

  const auto check_if_duplicate = [](const Point2d& p1, const Point2d& p2) {
    constexpr auto threshold = 1e-3;
    return (p1 - p2).norm() < threshold;
  };
  dataset.erase(std::unique(dataset.begin(), dataset.end(), check_if_duplicate), dataset.end());

  // Ensure k is at least 3 and no larger than dataset.size()-1.
  const auto kk = std::min(std::max(num_of_neighbor, 3UL), dataset.size() - 1);

  // Initialize hull with the first (lowest) point.
  auto first_point = dataset.front();
  std::vector<Point2d> hull{first_point};
  Point2d current_point = first_point;
  RemovePoint(dataset, first_point);
  double prev_angle = 0;
  size_t step = 2; // Step counter (starting at 2 as in pseudocode)

  // Main loop: continue until current_point equals first_point (closing the polygon) or dataset empties.
  while (!dataset.empty() && (current_point != first_point || step == 2)) {

    // At step 5, reinsert the first_point (to allow closure).
    if (step == 5) {
      dataset.push_back(first_point);
    }

    // Build a KDTree from the current dataset and get k nearest points.
    lane_departure_checker::KDTree tree(dataset);
    auto k_nearest_points = tree.kNearest(current_point, kk);

    // Sort the k-nearest candidate points by descending turning angle.
    auto c_points = SortByAngle(k_nearest_points, current_point, prev_angle);

    bool candidateFound = false;
    size_t candidateIndex = 0;

    // Iterate over candidates (starting at index 0).
    for (size_t i = 0; i < c_points.size(); i++) {
      bool intersects = false; // Reset intersection flag for this candidate.
      // Determine if candidate equals the first point.
      auto last_point = (c_points[i] == first_point) ? 1UL : 0UL;

      // Check if the candidate edge (from last hull vertex to candidate) intersects any existing hull edge.
      // Start with j = 2 (comparing the newest edge with earlier ones).
      for (size_t j = 2; j < hull.size() - last_point; j++) {
        // Create segments: current edge from hull[hull.size()-1] to candidate, and an edge from hull[hull.size()-1-j] to hull[hull.size()-j].
        auto p1 = autoware_utils::to_msg(hull[hull.size()-1].to_3d());
        auto p2 = autoware_utils::to_msg(c_points[i].to_3d());
        auto p3 = autoware_utils::to_msg(hull[hull.size()-1 - j].to_3d());
        auto p4 = autoware_utils::to_msg(hull[hull.size()-j].to_3d());
        auto intersect = autoware_utils::intersect(p1, p2, p3, p4);
        if (intersect.has_value()) {
          intersects = true;
          break;
        }
      }
      // If no intersections were found for this candidate, select it.
      if (!intersects) {
        candidateFound = true;
        candidateIndex = i;
        break;
      }
    }

    // If no valid candidate was found among the k-nearest, try with a higher k.
    if (!candidateFound) {
      return concave_hull(point_list, kk + 1);
    }

    // Update current_point with the chosen candidate.
    current_point = c_points[candidateIndex];
    hull.push_back(current_point);

    // Update previous angle using the last edge added.
    // (Using the last two points in hull: hull[hull.size()-2] to hull[hull.size()-1])
    prev_angle = computeCandidateAngle(hull[hull.size()-2], hull[hull.size()-1]);
    step++;

    // Remove the chosen candidate from the dataset.
    RemovePoint(dataset, current_point);

    // Check if all remaining points in dataset are inside the current hull.
    if (!allPointsInsidePolygon(dataset, hull)) {
      return concave_hull(point_list, kk + 1);
    }
  }
  return hull;
}
}  // namespace autoware::lane_departure_checker::utils
