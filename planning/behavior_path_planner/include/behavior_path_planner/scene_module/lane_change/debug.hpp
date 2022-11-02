// Copyright 2022 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__DEBUG_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__DEBUG_HPP_

#include "behavior_path_planner/debug_utilities.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace marker_utils::lane_change_markers
{
using behavior_path_planner::LaneChangePath;
using behavior_path_planner::ShiftLine;
using geometry_msgs::msg::Pose;
using marker_utils::CollisionCheckDebug;
using visualization_msgs::msg::MarkerArray;

MarkerArray showObjectInfo(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns);
MarkerArray showAllValidLaneChangePath(
  const std::vector<LaneChangePath> & lanes, std::string && ns);
MarkerArray showLerpedPose(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns);
MarkerArray showEgoPredictedPaths(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns);
MarkerArray showPolygon(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns);
MarkerArray showPolygonPose(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns);
MarkerArray show_shift_pose(const Pose & shift_pose, std::string && ns, const int32_t id = 0);
MarkerArray show_shift_line(const ShiftLine & shift_line, std::string && ns, const int32_t id = 0);
}  // namespace marker_utils::lane_change_markers
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__DEBUG_HPP_
