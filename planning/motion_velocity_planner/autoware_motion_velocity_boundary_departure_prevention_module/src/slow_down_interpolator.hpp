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

#include "type_alias.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <tl_expected/expected.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#ifndef SLOW_DOWN_INTERPOLATOR_HPP_
#define SLOW_DOWN_INTERPOLATOR_HPP_

namespace autoware::motion_velocity_planner
{

class SlowDownInterpolator
{
public:
  SlowDownInterpolator(
    std::shared_ptr<const PlannerData> planner_data, const double min_lat_dist_m,
    const double max_lat_dist_m, const double min_vel_mps, const double max_vel_mps,
    const double min_decel_mps2, const double max_decel_mps2)
  : planner_data_(std::move(planner_data)),
    th_min_lat_dist_m_(min_lat_dist_m),
    th_max_lat_dist_m_(max_lat_dist_m),
    th_min_vel_mps_(min_vel_mps),
    th_max_vel_mps_(max_vel_mps),
    th_min_decel_mps2_(min_decel_mps2),
    th_max_decel_mps2_(max_decel_mps2)
  {
    if ((th_max_vel_mps_ - th_min_vel_mps_) < 1e-1) {
      throw std::invalid_argument(
        "Fail to initialize SlowDownInterpolator. Min and max vel_mps difference is too small");
    }
    slope_ = (th_max_vel_mps_ - th_min_vel_mps_) / (th_max_lat_dist_m_ - th_min_lat_dist_m_);
  }

  [[nodiscard]] tl::expected<std::tuple<double, double, double>, std::string> get_interp_to_point(
    const double arclength_to_point_m, const double lat_dist_to_bound_m) const
  {
    const auto vel_mps = interp_velocity(lat_dist_to_bound_m);
    const auto exp_decel_mps2 = calc_expected_deceleration(vel_mps, arclength_to_point_m);

    if (std::abs(exp_decel_mps2) > std::abs(th_max_decel_mps2_)) {
      return tl::unexpected<std::string>(
        "Expected deceleration for the interpolated velocity exceeded threshold.");
    }

    const auto curr_speed_mps = std::abs(planner_data_->current_odometry.twist.twist.linear.x);
    const auto decel_mps2 = std::clamp(exp_decel_mps2, th_max_decel_mps2_, th_min_decel_mps2_);
    const auto expected_vel =
      std::sqrt(curr_speed_mps * curr_speed_mps + 2 * decel_mps2 * arclength_to_point_m);
    const auto rel_dist_m =
      (vel_mps * vel_mps - curr_speed_mps * curr_speed_mps) / (2 * decel_mps2);
    fmt::print(
      "vel {}, curr vel {}, decel{} rel dist n {}\n", expected_vel, curr_speed_mps, decel_mps2,
      rel_dist_m);

    return std::make_tuple(rel_dist_m, vel_mps, decel_mps2);
  }

private:
  [[nodiscard]] double calc_expected_deceleration(
    const double exp_vel, const double arclength_to_point_m) const
  {
    return (th_min_vel_mps_ * th_min_vel_mps_ - exp_vel * exp_vel) / (2 * arclength_to_point_m);
  }

  [[nodiscard]] double calc_dist_to_point(const Point2d & slow_down_point) const
  {
    return calc_dist_to_point(autoware_utils::to_msg(slow_down_point.to_3d(0.0)));
  }

  [[nodiscard]] double calc_dist_to_point(const geometry_msgs::msg::Point & slow_down_point) const
  {
    const auto & curr_point = planner_data_->current_odometry.pose.pose.position;
    return autoware_utils::calc_distance2d(curr_point, slow_down_point);
  }

  [[nodiscard]] double interp_velocity(const double lat_dist) const
  {
    const auto vel_mps = planner_data_->current_odometry.twist.twist.linear.x;

    if (lat_dist >= th_max_lat_dist_m_) {
      return vel_mps;
    }
    if (lat_dist <= th_min_lat_dist_m_) {
      return th_min_vel_mps_;
    }

    return std::max(th_min_vel_mps_, slope_ * vel_mps);
  }

  std::shared_ptr<const PlannerData> planner_data_;
  double th_min_lat_dist_m_{0.0};
  double th_max_lat_dist_m_{0.0};
  double th_min_vel_mps_{0.0};
  double th_max_vel_mps_{0.0};
  double th_min_decel_mps2_{0.0};
  double th_max_decel_mps2_{0.0};
  double slope_{0.0};
};

}  // namespace autoware::motion_velocity_planner

#endif  // SLOW_DOWN_INTERPOLATOR_HPP_
