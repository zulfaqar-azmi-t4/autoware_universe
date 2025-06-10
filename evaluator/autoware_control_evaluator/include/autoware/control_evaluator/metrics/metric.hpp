// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__CONTROL_EVALUATOR__METRICS__METRIC_HPP_
#define AUTOWARE__CONTROL_EVALUATOR__METRICS__METRIC_HPP_

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace control_diagnostics
{
/**
 * @brief Enumeration of trajectory metrics
 */
enum class Metric {
  velocity,
  acceleration,
  jerk,
  lateral_deviation,
  yaw_deviation,
  goal_longitudinal_deviation,
  goal_lateral_deviation,
  goal_yaw_deviation,
  left_boundary_distance,
  right_boundary_distance,
  left_uncrossable_boundary_distance,
  right_uncrossable_boundary_distance,
  steering_angle,
  steering_rate,
  steering_acceleration,
  stop_deviation,
  SIZE,
};

static const std::unordered_map<std::string, Metric> str_to_metric = {
  {"velocity", Metric::velocity},
  {"acceleration", Metric::acceleration},
  {"jerk", Metric::jerk},
  {"lateral_deviation", Metric::lateral_deviation},
  {"yaw_deviation", Metric::yaw_deviation},
  {"goal_longitudinal_deviation", Metric::goal_longitudinal_deviation},
  {"goal_lateral_deviation", Metric::goal_lateral_deviation},
  {"goal_yaw_deviation", Metric::goal_yaw_deviation},
  {"left_boundary_distance", Metric::left_boundary_distance},
  {"right_boundary_distance", Metric::right_boundary_distance},
  {"left_uncrossable_boundary_distance", Metric::left_uncrossable_boundary_distance},
  {"right_uncrossable_boundary_distance", Metric::right_uncrossable_boundary_distance},
  {"steering_angle", Metric::steering_angle},
  {"steering_rate", Metric::steering_rate},
  {"steering_acceleration", Metric::steering_acceleration},
  {"stop_deviation", Metric::stop_deviation},
};

static const std::unordered_map<Metric, std::string> metric_to_str = {
  {Metric::velocity, "velocity"},
  {Metric::acceleration, "acceleration"},
  {Metric::jerk, "jerk"},
  {Metric::lateral_deviation, "lateral_deviation"},
  {Metric::yaw_deviation, "yaw_deviation"},
  {Metric::goal_longitudinal_deviation, "goal_longitudinal_deviation"},
  {Metric::goal_lateral_deviation, "goal_lateral_deviation"},
  {Metric::goal_yaw_deviation, "goal_yaw_deviation"},
  {Metric::left_boundary_distance, "left_boundary_distance"},
  {Metric::right_boundary_distance, "right_boundary_distance"},
  {Metric::left_uncrossable_boundary_distance, "left_uncrossable_boundary_distance"},
  {Metric::right_uncrossable_boundary_distance, "right_uncrossable_boundary_distance"},
  {Metric::steering_angle, "steering_angle"},
  {Metric::steering_rate, "steering_rate"},
  {Metric::steering_acceleration, "steering_acceleration"},
  {Metric::stop_deviation, "stop_deviation"},
};

// Metrics descriptions
static const std::unordered_map<Metric, std::string> metric_descriptions = {
  {Metric::velocity, "Velocity[m/s]"},
  {Metric::acceleration, "Acceleration[m/s^2]"},
  {Metric::jerk, "Jerk[m/s^3]"},
  {Metric::lateral_deviation, "Lateral deviation from the reference trajectory[m]"},
  {Metric::yaw_deviation, "Yaw deviation from the reference trajectory[rad]"},
  {Metric::goal_longitudinal_deviation, "Longitudinal deviation from the goal point[m]"},
  {Metric::goal_lateral_deviation, "Lateral deviation from the goal point[m]"},
  {Metric::goal_yaw_deviation, "Yaw deviation from the goal point[rad]"},
  {Metric::left_boundary_distance, "Signed distance to the left boundary[m]"},
  {Metric::right_boundary_distance, "Signed distance to the right boundary[m]"},
  {Metric::left_uncrossable_boundary_distance,
   "Signed distance to the left uncrossable boundary[m]"},
  {Metric::right_uncrossable_boundary_distance,
   "Signed distance to the right uncrossable boundary[m]"},
  {Metric::steering_angle, "Steering angle[rad]"},
  {Metric::steering_rate, "Steering angle rate[rad/s]"},
  {Metric::steering_acceleration, "Steering angle acceleration[rad/s^2]"},
  {Metric::stop_deviation, "Deviation to the stop line when the ego stop by a module[m]"},
};

namespace details
{
static struct CheckCorrectMaps
{
  CheckCorrectMaps()
  {
    if (
      str_to_metric.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_to_str.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_descriptions.size() != static_cast<size_t>(Metric::SIZE)) {
      std::cerr << "[metrics/metrics.hpp] Maps are not defined for all metrics: ";
      std::cerr << str_to_metric.size() << " " << metric_to_str.size() << " "
                << metric_descriptions.size() << std::endl;
    }
  }
} check;

}  // namespace details
}  // namespace control_diagnostics

#endif  // AUTOWARE__CONTROL_EVALUATOR__METRICS__METRIC_HPP_
