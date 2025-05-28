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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_

#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker
{
enum class DepartureType {
  NONE = 0,
  NEAR_BOUNDARY,
  APPROACHING_DEPARTURE,
  CRITICAL_DEPARTURE,
  UNKNOWN
};

struct FootprintMargin
{
  double lon_m{0.0};
  double lat_m{0.0};

  FootprintMargin operator+(const FootprintMargin & other) const
  {
    return FootprintMargin{lon_m + other.lon_m, lat_m + other.lat_m};
  }
};

struct LonTracking
{
  double scale{1.0};
  double extra_margin_m{0.25};
};

template <typename T>
struct Side
{
  T right;
  T left;
  T & operator[](const std::string_view key)
  {
    if (key == "left") return left;
    if (key == "right") return right;
    throw std::out_of_range(std::string("Invalid key: ") + std::string(key));
  }

  const T & operator[](const std::string_view key) const
  {
    if (key == "left") return left;
    if (key == "right") return right;
    throw std::out_of_range(std::string("Invalid key: ") + std::string(key));
  }
};

template <typename T>
struct SideExt : Side<T>
{
  Pose pose;
  double dist_from_start{0.0};
};

struct Projection
{
  Point2d orig;
  Point2d proj;
  double dist{std::numeric_limits<double>::max()};
};

struct ProjectionWithSegment
{
  Projection projection;
  Segment2d nearest_segment;
  size_t idx_from_ego_sides_from_footprints{0};
  ProjectionWithSegment() = default;
  ProjectionWithSegment(Projection proj, Segment2d seg, size_t idx)
  : projection(std::move(proj)),
    nearest_segment(std::move(seg)),
    idx_from_ego_sides_from_footprints(idx)
  {
  }
};

using SideProjOpt = Side<std::optional<Projection>>;
using BoundarySide = Side<std::vector<Segment2d>>;
using BoundarySideWithIdx = Side<std::vector<SegmentWithIdx>>;
using SideToBoundPojections = Side<std::vector<ProjectionWithSegment>>;
using EgoSide = SideExt<Segment2d>;
using EgoSides = std::vector<EgoSide>;

struct DeparturePoint
{
  std::string uuid;
  DepartureType type = DepartureType::NONE;
  Point2d point;
  std::string_view direction;
  double th_dist_hysteresis{2.0};
  double th_lat_dist_to_bounday_hyteresis{0.01};
  double lat_dist_to_bound{1000.0};
  double dist_on_traj{1000.0};
  double dist_from_ego{0.0};
  double velocity{0.0};
  bool can_be_removed{false};
  TrajectoryPoint point_on_prev_traj;

  [[nodiscard]] bool is_nearby(const Pose & pose) const { return is_nearby(pose.position); }

  [[nodiscard]] bool is_nearby(const Point & point) const { return is_nearby({point.x, point.y}); }

  [[nodiscard]] bool is_close_to_bound() const
  {
    return (
      (type == DepartureType::CRITICAL_DEPARTURE || type == DepartureType::APPROACHING_DEPARTURE) &&
      std::abs(lat_dist_to_bound) < th_lat_dist_to_bounday_hyteresis);
  }

  [[nodiscard]] bool is_nearby(const Point2d & candidate_point) const
  {
    const auto diff = boost::geometry::distance(point, candidate_point);

    return !is_close_to_bound() && diff < th_dist_hysteresis;
  }

  [[nodiscard]] Point to_geom_pt(const double z = 0.0) const
  {
    return autoware_utils::to_msg(point.to_3d(z));
  }

  bool operator<(const DeparturePoint & other) const { return dist_on_traj < other.dist_on_traj; }
};

using DeparturePoints = std::vector<DeparturePoint>;

struct DepartureInterval
{
  TrajectoryPoint start;
  TrajectoryPoint end;
  std::string_view direction;
  double start_dist_on_traj;
  double end_dist_on_traj;

  bool start_at_traj_front{false};

  DeparturePoints candidates;
};
using DepartureIntervals = std::vector<DepartureInterval>;

struct Param
{
  int th_max_lateral_query_num{5};
  double footprint_extra_margin{};
  FootprintMargin footprint_envelop;
  std::vector<std::string> boundary_types_to_detect;
  LonTracking lon_tracking;
};

struct Input
{
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom;
  lanelet::LaneletMapPtr lanelet_map;
  LaneletRoute::ConstSharedPtr route;
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets shoulder_lanelets;
  Trajectory::ConstSharedPtr reference_trajectory;
  Trajectory::ConstSharedPtr predicted_trajectory;
  std::vector<std::string> boundary_types_to_detect;
};

using FootprintWithPose = std::vector<std::pair<LinearRing2d, Pose>>;

struct BDCData
{
  FootprintWithPose normal_fp;
  FootprintWithPose ab_enveloped_fp;
  FootprintWithPose ab_lon_tracking_fp;
  FootprintWithPose ab_steering_fp;
  EgoSides ego_sides_from_footprints;
  BoundarySideWithIdx boundary_segments;
  SideToBoundPojections side_to_bound_projections;
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
