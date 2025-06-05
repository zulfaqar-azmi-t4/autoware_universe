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
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <magic_enum.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <optional>
#include <string>
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

enum class AbnormalityType { NORMAL, LOCALIZATION, LONGITUDINAL, STEERING };
template <typename T>
struct Abnormalities
{
  T normal;
  T longitudinal;
  T localization;
  T steering;
  T & operator[](const AbnormalityType key)
  {
    if (key == AbnormalityType::NORMAL) return normal;
    if (key == AbnormalityType::LOCALIZATION) return localization;
    if (key == AbnormalityType::LONGITUDINAL) return longitudinal;
    if (key == AbnormalityType::STEERING) return steering;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  const T & operator[](const AbnormalityType key) const
  {
    if (key == AbnormalityType::NORMAL) return normal;
    if (key == AbnormalityType::LOCALIZATION) return localization;
    if (key == AbnormalityType::LONGITUDINAL) return longitudinal;
    if (key == AbnormalityType::STEERING) return steering;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }
};

enum class SideKey { LEFT, RIGHT };
constexpr std::array<SideKey, 2> g_side_keys = {SideKey::LEFT, SideKey::RIGHT};

template <typename T>
struct Side
{
  T right;
  T left;
  T & operator[](const SideKey key)
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  const T & operator[](const SideKey key) const
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }
};

template <typename T>
struct SideExt : Side<T>
{
  Pose pose;
  double dist_from_start{0.0};
};

struct ProjectionToBound
{
  Point2d pt_on_ego;    // orig
  Point2d pt_on_bound;  // proj
  Segment2d nearest_bound_seg;
  double lat_dist{std::numeric_limits<double>::max()};
  size_t ego_sides_idx{0};
  ProjectionToBound() = default;
  ProjectionToBound(Point2d pt_on_ego, Point2d pt_on_bound, Segment2d seg, double dist, size_t idx)
  : pt_on_ego(std::move(pt_on_ego)),
    pt_on_bound(std::move(pt_on_bound)),
    nearest_bound_seg(std::move(seg)),
    lat_dist(dist),
    ego_sides_idx(idx)
  {
  }
};

using BoundarySide = Side<std::vector<Segment2d>>;
using BoundarySideWithIdx = Side<std::vector<SegmentWithIdx>>;
using ProjectionsToBound = Side<std::vector<ProjectionToBound>>;
using EgoSide = SideExt<Segment2d>;
using EgoSides = std::vector<EgoSide>;
using DepartureTypeIdx = std::pair<DepartureType, size_t>;
using DepartureStatuses = Side<std::vector<DepartureTypeIdx>>;

struct DeparturePoint
{
  std::string uuid;
  DepartureType type = DepartureType::NONE;
  SideKey direction = SideKey::LEFT;
  Point2d point;
  double th_dist_hysteresis{2.0};
  double th_lat_dist_to_bounday_hyteresis{0.01};
  double lat_dist_to_bound{1000.0};
  double dist_on_traj{1000.0};
  double dist_from_ego{0.0};
  double velocity{0.0};
  bool can_be_removed{false};

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

struct CriticalDeparturePoint : DeparturePoint
{
  TrajectoryPoint point_on_prev_traj;
  CriticalDeparturePoint() = default;
  explicit CriticalDeparturePoint(const DeparturePoint & base)
  {
    uuid = base.uuid;
    type = base.type;
    point = base.point;
    direction = base.direction;
    th_dist_hysteresis = base.th_dist_hysteresis;
    th_lat_dist_to_bounday_hyteresis = base.th_lat_dist_to_bounday_hyteresis;
    lat_dist_to_bound = base.lat_dist_to_bound;
    dist_on_traj = base.dist_on_traj;
    dist_from_ego = base.dist_from_ego;
    velocity = base.velocity;
    can_be_removed = base.can_be_removed;
  }
};

using CriticalDeparturePoints = std::vector<CriticalDeparturePoint>;

struct DepartureInterval
{
  TrajectoryPoint start;
  TrajectoryPoint end;
  SideKey direction;
  double start_dist_on_traj;
  double end_dist_on_traj;

  bool start_at_traj_front{false};
  bool has_merged{false};

  DeparturePoints candidates;
};
using DepartureIntervals = std::vector<DepartureInterval>;

struct Param
{
  int th_max_lateral_query_num{5};
  double footprint_extra_margin{};
  FootprintMargin footprint_envelop;
  std::vector<std::string> boundary_types_to_detect;
  std::vector<AbnormalityType> abnormality_types_to_compensate;
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

using Footprint = LinearRing2d;
using Footprints = std::vector<Footprint>;
using PoseWithDist = std::pair<Pose, double>;

struct BDCData
{
  Abnormalities<EgoSides> footprints_sides;
  Abnormalities<Footprints> footprints;
  Abnormalities<ProjectionsToBound> projections_to_bound;
  BoundarySideWithIdx boundary_segments;
  ProjectionsToBound min_projections_to_bound;
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
