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

#include "debug.hpp"

#include "str_map.hpp"
#include "type_alias.hpp"

#include <std_msgs/msg/detail/color_rgba__struct.hpp>

#include <string>
#include <vector>

namespace color
{
using std_msgs::msg::ColorRGBA;

inline ColorRGBA blue(float a = 0.99)
{
  return autoware_utils::create_marker_color(0., 0., 1., a);
}

inline ColorRGBA yellow(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 1., 0., a);
}

inline ColorRGBA green(float a = 0.99)
{
  return autoware_utils::create_marker_color(0., 1., 0., a);
}

inline ColorRGBA aqua(float a = 0.99)
{
  return autoware_utils::create_marker_color(0., 1., 1., a);
}

inline ColorRGBA magenta(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 0., 1., a);
}

inline ColorRGBA medium_orchid(float a = 0.99)
{
  return autoware_utils::create_marker_color(0.729, 0.333, 0.827, a);
}

inline ColorRGBA light_pink(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 0.713, 0.756, a);
}

inline ColorRGBA light_yellow(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 1., 0.878, a);
}

inline ColorRGBA light_steel_blue(float a = 0.99)
{
  return autoware_utils::create_marker_color(0.690, 0.768, 0.870, a);
}

inline ColorRGBA white(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 1., 1., a);
}

inline ColorRGBA grey(float a = 0.99)
{
  return autoware_utils::create_marker_color(.5, .5, .5, a);
}
}  // namespace color

namespace autoware::motion_velocity_planner::debug
{
Marker create_ego_sides_marker(
  const EgoSides & ego_sides_from_footprints, Marker marker, std::string && ns,
  const double base_link_z)
{
  marker.ns = ns;
  marker.points.reserve(ego_sides_from_footprints.size() * 4);
  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };
  for (const auto & ego_footprint_sides : ego_sides_from_footprints) {
    const auto & left = ego_footprint_sides.left;
    marker.points.push_back(to_geom(left.first));
    marker.points.push_back(to_geom(left.second));
    const auto & right = ego_footprint_sides.right;
    marker.points.push_back(to_geom(right.first));
    marker.points.push_back(to_geom(right.second));
  }

  return marker;
}

Marker create_side_to_boundary_marker(
  const std::vector<ProjectionWithSegment> & side_to_boundary, Marker marker, std::string && ns,
  const double base_link_z)
{
  marker.ns = ns;
  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };
  for (const auto & [projection, segment, dist_from_start] : side_to_boundary) {
    const auto & [orig, proj, dist] = projection;
    marker.color = color::blue();
    marker.points.push_back(to_geom(orig));
    marker.points.push_back(to_geom(proj));
    marker.points.push_back(to_geom(segment.first));
    marker.points.push_back(to_geom(segment.second));
  }
  return marker;
}

Marker create_departure_points_marker(
  const DeparturePoints & departure_points, const rclcpp::Time & curr_time,
  const double base_link_z)
{
  int32_t id{0};
  auto marker = create_default_marker(
    "map", curr_time, "departure_points", ++id, visualization_msgs::msg::Marker::SPHERE_LIST,
    create_marker_scale(0.25, 0.25, 1.0), color::yellow());
  for (const auto & pt : departure_points) {
    marker.points.push_back(pt.to_geom_pt(base_link_z));
  }
  return marker;
}

Marker create_footprint_marker(
  const Footprints & footprints, const rclcpp::Time & curr_time,
  const std::string_view abnormality_type, const double base_link_z,
  const std_msgs::msg::ColorRGBA & color)
{
  int32_t id{0};
  auto marker_ll = create_default_marker(
    "map", curr_time, "footprint_" + std::string(abnormality_type), id,
    visualization_msgs::msg::Marker::LINE_LIST, create_marker_scale(0.05, 0, 0), color);
  if (!footprints.empty()) {
    marker_ll.points.reserve(footprints.size() * footprints.front().size());
  }

  for (const auto & footprint : footprints) {
    for (size_t i = 0; i + 1 < footprint.size(); ++i) {
      const auto & p1 = footprint.at(i);
      const auto & p2 = footprint.at(i + 1);

      marker_ll.points.push_back(autoware_utils::to_msg(p1.to_3d(base_link_z)));
      marker_ll.points.push_back(autoware_utils::to_msg(p2.to_3d(base_link_z)));
    }
  }

  return marker_ll;
}

Marker create_boundary_segments_marker(
  const BoundarySideWithIdx & boundaries, Marker marker, std::string && ns,
  const double base_link_z)
{
  marker.ns = ns;

  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };
  marker.color = color::medium_orchid();
  for (const auto & [segment, ll_id, idx_curr, idx_next] : boundaries.left) {
    marker.points.push_back(to_geom(segment.first));
    marker.points.push_back(to_geom(segment.second));
  }
  for (const auto & [segment, ll_id, idx_curr, idx_next] : boundaries.right) {
    marker.points.push_back(to_geom(segment.first));
    marker.points.push_back(to_geom(segment.second));
  }
  return marker;
}

MarkerArray create_slow_down_interval(
  const std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> &
    slow_down_points,
  const rclcpp::Time & curr_time)
{
  int32_t id{0};
  auto marker_1 = create_default_marker(
    "map", curr_time, "start_slow", id, visualization_msgs::msg::Marker::POINTS,
    create_marker_scale(0.25, 0.25, 1.0), color::light_steel_blue());

  auto marker_2 = create_default_marker(
    "map", curr_time, "stop_slow", id, visualization_msgs::msg::Marker::POINTS,
    create_marker_scale(0.25, 0.25, 1.0), color::light_pink());
  for (const auto & [start, stop] : slow_down_points) {
    marker_1.points.push_back(start);
    marker_2.points.push_back(stop);
  }

  MarkerArray marker_array;
  marker_array.markers = {marker_1, marker_2};
  return marker_array;
}

Marker create_departure_interval_marker(
  const DepartureIntervals & departure_intervals, Marker marker, std::string && ns)
{
  marker.ns = ns;
  marker.color = color::magenta();
  for (const auto & departure_interval : departure_intervals) {
    marker.points.push_back(departure_interval.start.pose.position);
    marker.points.push_back(departure_interval.end.pose.position);
  }
  return marker;
}

MarkerArray create_debug_marker_array(
  const param::Output & output, const rclcpp::Clock::SharedPtr & clock_ptr,
  const double base_link_z)
{
  const auto line_list = visualization_msgs::msg::Marker::LINE_LIST;
  const auto curr_time = clock_ptr->now();
  const auto color = color::green();
  const auto m_scale = create_marker_scale(0.05, 0, 0);

  MarkerArray marker_array;

  auto marker = create_default_marker("map", curr_time, "", 0, line_list, m_scale, color);

  marker_array.markers.push_back(
    create_ego_sides_marker(output.ego_sides_from_footprints, marker, "ego_sides", base_link_z));
  marker_array.markers.push_back(create_side_to_boundary_marker(
    output.side_to_bound_projections.left, marker, "closest_to_left_side", base_link_z));
  marker_array.markers.push_back(create_side_to_boundary_marker(
    output.side_to_bound_projections.right, marker, "closest_to_right_side", base_link_z));
  marker_array.markers.push_back(
    create_departure_points_marker(output.departure_points, curr_time, base_link_z));
  marker_array.markers.push_back(create_boundary_segments_marker(
    output.boundary_segments, marker, "boundary_segments", base_link_z));
  marker_array.markers.push_back(
    create_departure_interval_marker(output.departure_intervals, marker, "departure interval"));
  for (const std::string_view type : abnormality_keys) {
    marker_array.markers.push_back(create_footprint_marker(
      output.footprints[type], curr_time, type, base_link_z, color::aqua()));
  }
  autoware_utils::append_marker_array(
    create_slow_down_interval(output.slow_down_interval, curr_time), &marker_array);

  return marker_array;
}

}  // namespace autoware::motion_velocity_planner::debug
