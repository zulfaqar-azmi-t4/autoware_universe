// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__COLLISION_DETECTOR__TYPES_HPP_
#define AUTOWARE__COLLISION_DETECTOR__TYPES_HPP_

#include <tl_expected/expected.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <utility>

namespace autoware::collision_detector
{
using Obstacle = std::pair<double /* distance */, geometry_msgs::msg::Point>;

/// @brief Reason why an obstacle search returned no obstacle.
enum class ObstacleSearchError {
  transform_unavailable,  ///< the search could not run, so the result is unknown
  no_obstacle_found       ///< the search ran and found nothing
};

using result_t = tl::expected<Obstacle, ObstacleSearchError>;
}  // namespace autoware::collision_detector

#endif  // AUTOWARE__COLLISION_DETECTOR__TYPES_HPP_
