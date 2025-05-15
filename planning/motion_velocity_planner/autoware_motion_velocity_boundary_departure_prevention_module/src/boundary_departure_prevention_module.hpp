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

#ifndef BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_
#define BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_

#include "parameters.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <tl_expected/expected.hpp>

#include <fmt/format.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{
class BoundaryDeparturePreventionModule : public PluginModuleInterface
{
public:
  void init(rclcpp::Node & node, const std::string & module_name) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  VelocityPlanningResult plan(
    const TrajectoryPoints & raw_trajectory_points,
    const TrajectoryPoints & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;
  std::string get_module_name() const override { return module_name_; };

private:
  void subscribe_topics(rclcpp::Node & node);
  void publish_topics(rclcpp::Node & node);
  tl::expected<param::Output, std::string> plan(
    const PoseWithCovariance & pose_with_covariance, const double abs_velocity,
    const TrajectoryPoints & ego_pred_traj, const double footprint_margin_scale);
  [[nodiscard]] bool is_data_ready(std::unordered_map<std::string, double> & processing_times);
  [[nodiscard]] bool is_data_valid() const;
  [[nodiscard]] bool is_data_timeout(const Odometry & odom) const;

  bool found_nearby_points(
    const Point2d & candidate_point, DeparturePoints & curr_departure_points);

  void remove_exist_point(
    const std::vector<ProjectionWithSegment> & projections, DeparturePoints & departure_points,
    std::vector<param::DepartureTypeIdx> & statuses);

  void check_departure_points_lifetime();

  bool is_critical_departing_{false};
  std::string module_name_;
  param::Output output_;
  param::NodeParam node_param_;
  rclcpp::Clock::SharedPtr clock_ptr_;
  rclcpp::TimerBase::SharedPtr timer_ptr_;
  MarkerArray slow_down_wall_marker_;
  static constexpr auto throttle_duration_ms{5000};

  StopWatch<std::chrono::milliseconds> departure_points_lifetime_watch_;

  Trajectory::ConstSharedPtr ego_pred_traj_ptr_;
  Control::ConstSharedPtr control_cmd_ptr_;
  OperationModeState::ConstSharedPtr op_mode_state_ptr_;
  std::unordered_map<std::string, double> processing_times_ms_;

  rclcpp::Subscription<Trajectory>::SharedPtr sub_ego_pred_traj_;
  rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_op_mode_state_;
  std::unique_ptr<BoundaryDepartureChecker> boundary_departure_checker_ptr_;

  std::unique_ptr<diagnostic_updater::Updater> updater_ptr_;
};
}  // namespace autoware::motion_velocity_planner
#endif  // BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_
