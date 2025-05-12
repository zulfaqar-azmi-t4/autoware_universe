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
  double lon;
  double lat;
};

template <typename T>
struct Side
{
  T left;
  T right;
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
  DepartureType type = DepartureType::NONE;
  double velocity{0.0};
  double lifetime{0.0};
  double th_dist_hysteresis{2.0};
  double th_lifetime{1.0};
  Point2d point;

  [[nodiscard]] bool is_nearby(const Pose & pose) const { return is_nearby(pose.position); }

  [[nodiscard]] bool is_nearby(const Point & point) const { return is_nearby({point.x, point.y}); }

  [[nodiscard]] bool is_nearby(const Point2d & candidate_point) const
  {
    const auto diff = boost::geometry::distance(point, candidate_point);
    return diff < th_dist_hysteresis;
  }

  [[nodiscard]] bool is_alive() const { return lifetime <= th_lifetime; }
};
using DeparturePoints = std::unordered_map<std::string, DeparturePoint>;
struct Param
{
  double footprint_margin_scale{};
  double footprint_extra_margin{};
  double resample_interval{};
  double max_deceleration{};
  double delay_time{};
  double min_braking_distance{};
  // nearest search to ego
  double ego_nearest_dist_threshold{};
  double ego_nearest_yaw_threshold{};
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
  FootprintWithPose fp_with_pose;
  EgoSides ego_sides_from_footprints;
  BoundarySideWithIdx boundary_segments;
  SideToBoundPojections side_to_bound_projections;
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
