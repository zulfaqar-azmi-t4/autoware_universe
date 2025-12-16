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
#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware::manual_trajectory
{
constexpr double g_epsilon = 1e-6;
constexpr double g_safety_timeout = 100.0;  // [s] Watchdog for the generation loop
constexpr double g_min_velocity_threshold = 1.0 / 3.6;
constexpr double g_min_start_speed = 1.0 / 3.6;  // [m/s] Avoid zero start speed

enum class TrajectoryMode { NORMAL, STOP_AND_GO };

struct Constraint
{
  double min_acc{-1.0};   // [m/s^2]
  double max_acc{1.0};    // [m/s^2]
  double min_jerk{-1.0};  // [m/s^3]
  double max_jerk{1.0};   // [m/s^3]
};

struct TrajectoryGenerationParams
{
  TrajectoryMode mode = TrajectoryMode::NORMAL;
  double time_step = 0.1;            // [s] dt
  double duration = 10.0;            // [s] Horizon
  double map_velocity_limit = 20.0;  // [m/s] Fallback if path has no limit
                                     //
  double move_duration = 5.0;        // [s] Drive for this long
  double stop_duration = 5.0;        // [s] Then stop for this long

  double system_delay = 0.5;  // [s] NEW: Compensate for reaction time
  double temporary_stop_distance = -1.0;

  Constraint normal;
  Constraint limit;
};

struct VelocityPoint
{
  double s{0.0};
  double v{0.0};
  double a{0.0};
  double v_limit{0.0};  // Map/Path limit
};

struct SpatialProfile
{
  std::vector<VelocityPoint> points;
  std::vector<double> s_values;  // Cached accumulated distances for fast search
  double total_length{0.0};
};

enum class TestState {
  IDLE,       // Initial State
  DRIVING_1,  // First 5 seconds
  BRAKING,    // Decelerating to stop
  STOPPED,    // Waiting for 5 seconds
  DRIVING_2   // Resuming
};
}  // namespace autoware::manual_trajectory

#endif  // DATA_STRUCTS_HPP_
