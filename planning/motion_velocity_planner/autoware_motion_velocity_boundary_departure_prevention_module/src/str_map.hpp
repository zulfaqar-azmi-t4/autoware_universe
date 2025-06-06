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

#ifndef STR_MAP_HPP_
#define STR_MAP_HPP_

namespace autoware::motion_velocity_planner
{
static constexpr const char * init_bdc_ptr = "init_bdc_ptr";
static constexpr const char * convert_ego_pred_to_aw_traj = "convert_ego_pred_to_aw_traj";
static constexpr const char * get_departure_points = "get_departure_points";
static constexpr const char * find_slow_down_points = "find_slow_down_points";
static constexpr const char * process_critical_departure = "process_critical_departure";
static constexpr const char * update_departure_interval = "update_departure_interval";
static constexpr const char * find_slow_down_intervals = "find_slow_down_intervals";
}  // namespace autoware::motion_velocity_planner

#endif  // STR_MAP_HPP_
