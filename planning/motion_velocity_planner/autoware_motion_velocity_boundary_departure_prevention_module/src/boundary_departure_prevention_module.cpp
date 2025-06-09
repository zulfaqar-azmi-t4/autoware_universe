// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "boundary_departure_prevention_module.hpp"

#include "debug.hpp"
#include "slow_down_interpolator.hpp"
#include "utils.hpp"

#include <autoware/boundary_departure_checker/utils.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <magic_enum.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

void BoundaryDeparturePreventionModule::init(
  rclcpp::Node & node, [[maybe_unused]] const std::string & module_name)
{
  module_name_ = module_name;
  clock_ptr_ = node.get_clock();
  logger_ = node.get_logger();

  node_param_ = NodeParam(node);
  subscribe_topics(node);
  publish_topics(node);

  updater_ptr_ = std::make_unique<diagnostic_updater::Updater>(&node);
  updater_ptr_->setHardwareID("motion_velocity_boundary_departure_prevention");
  updater_ptr_->add(
    "boundary_departure", [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {
      using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
      int8_t lvl{DiagStatus::OK};
      std::string msg{"OK"};

      if (output_.is_critical_departing) {
        lvl = DiagStatus::ERROR;
        msg = "vehicle will leave lane";
      }

      stat.summary(lvl, msg);
    });
}

void BoundaryDeparturePreventionModule::update_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  auto & pp = node_param_;

  const std::string module_name{"boundary_departure_prevention."};
  update_param(parameters, module_name + "th_data_timeout_s", pp.th_data_timeout_s);
  update_param(
    parameters, module_name + "boundary_types_to_detect", pp.bdc_param.boundary_types_to_detect);
  update_param(
    parameters, module_name + "th_max_lateral_query_num", pp.bdc_param.th_max_lateral_query_num);

  [[maybe_unused]] const auto abnormality_params = [&module_name, &parameters, &pp]() {
    const std::string ns_abnormality{module_name + "abnormality."};
    const std::string ns_normal_abnormality{ns_abnormality + "normal."};
    const std::string ns_steering_abnormality{ns_abnormality + "steering."};
    const std::string ns_localization_abnormality{ns_abnormality + "localization."};
    const std::string ns_longitudinal_abnormality{ns_abnormality + "longitudinal."};

    const auto has_type = [&](const AbnormalityType type_to_check) {
      return std::any_of(
        pp.bdc_param.abnormality_types_to_compensate.begin(),
        pp.bdc_param.abnormality_types_to_compensate.end(),
        [&](const AbnormalityType type) { return type == type_to_check; });
    };

    std::vector<AbnormalityType> abnormality_types_to_compensate;
    AbnormalitiesConfigs configs;

    auto compensate_normal = has_type(AbnormalityType::NORMAL);
    update_param(parameters, ns_normal_abnormality + "enable", compensate_normal);
    if (compensate_normal) {
      abnormality_types_to_compensate.emplace_back(AbnormalityType::NORMAL);

      NormalConfig normal_config;
      const std::string footprint_envelop_ns{ns_normal_abnormality + "footprint_envelop."};

      update_param(
        parameters, footprint_envelop_ns + "lat_m", normal_config.footprint_envelop.lat_m);
      update_param(
        parameters, footprint_envelop_ns + "lon_m", normal_config.footprint_envelop.lon_m);
      configs.insert({AbnormalityType::NORMAL, normal_config});
    }

    auto compensate_steering = has_type(AbnormalityType::STEERING);
    update_param(parameters, ns_steering_abnormality + "enable", compensate_steering);
    if (compensate_steering) {
      SteeringConfig steering_config;
      abnormality_types_to_compensate.emplace_back(AbnormalityType::STEERING);
      update_param(
        parameters, ns_steering_abnormality + "steering_rate_rps",
        steering_config.steering_rate_rps);
      configs.insert({AbnormalityType::STEERING, steering_config});
    }

    auto compensate_localization = has_type(AbnormalityType::LOCALIZATION);
    update_param(parameters, ns_localization_abnormality + "enable", compensate_localization);

    if (compensate_localization) {
      abnormality_types_to_compensate.emplace_back(AbnormalityType::LOCALIZATION);
      LocalizationConfig localization_config;
      const std::string footprint_envelop_ns{ns_localization_abnormality + "footprint_envelop."};
      update_param(
        parameters, footprint_envelop_ns + "lat_m", localization_config.footprint_envelop.lat_m);
      update_param(
        parameters, footprint_envelop_ns + "lon_m", localization_config.footprint_envelop.lon_m);
      configs.insert({AbnormalityType::LOCALIZATION, localization_config});
    }

    auto compensate_longitudinal = has_type(AbnormalityType::LONGITUDINAL);
    update_param(parameters, ns_longitudinal_abnormality + "enable", compensate_longitudinal);
    if (compensate_longitudinal) {
      abnormality_types_to_compensate.emplace_back(AbnormalityType::LONGITUDINAL);
      LongitudinalConfig longitudinal_config;
      const std::string lon_tracking_ns{ns_longitudinal_abnormality + "lon_tracking."};

      update_param(parameters, lon_tracking_ns + "scale", longitudinal_config.lon_tracking.scale);

      update_param(
        parameters, lon_tracking_ns + "extra_margin_m",
        longitudinal_config.lon_tracking.extra_margin_m);
      configs.insert({AbnormalityType::LONGITUDINAL, longitudinal_config});
    }

    pp.bdc_param.abnormality_types_to_compensate = abnormality_types_to_compensate;
    pp.bdc_param.abnormality_configs = configs;
  };
}

void BoundaryDeparturePreventionModule::subscribe_topics(rclcpp::Node & node)
{
  sub_ego_pred_traj_ = node.create_subscription<Trajectory>(
    "/control/trajectory_follower/lateral/predicted_trajectory", rclcpp::QoS{1},
    [&](const Trajectory::ConstSharedPtr msg) { ego_pred_traj_ptr_ = msg; });

  sub_control_cmd_ = node.create_subscription<Control>(
    "/control/command/control_cmd", rclcpp::QoS{1},
    [&](const Control::ConstSharedPtr msg) { control_cmd_ptr_ = msg; });
  sub_steering_angle_ = node.create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1},
    [&](const SteeringReport::ConstSharedPtr msg) { steering_angle_ptr_ = msg; });

  sub_op_mode_state_ = node.create_subscription<OperationModeState>(
    "~/api/operation_mode/state", rclcpp::QoS{1},
    [this](const OperationModeState::ConstSharedPtr msg) { op_mode_state_ptr_ = msg; });
}

void BoundaryDeparturePreventionModule::publish_topics(rclcpp::Node & node)
{
  const std::string ns = "boundary_departure_prevention";
  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns + "/debug_markers", 1);
}

VelocityPlanningResult BoundaryDeparturePreventionModule::plan(
  [[maybe_unused]] const TrajectoryPoints & raw_trajectory_points,
  [[maybe_unused]] const TrajectoryPoints & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  const auto & vehicle_info = planner_data->vehicle_info_;
  const auto & ll_map_ptr = planner_data->route_handler->getLaneletMapPtr();

  StopWatch<std::chrono::milliseconds> stopwatch_ms;
  stopwatch_ms.tic("init_bdc_ptr");
  if (!boundary_departure_checker_ptr_) {
    boundary_departure_checker_ptr_ =
      std::make_unique<BoundaryDepartureChecker>(ll_map_ptr, vehicle_info, node_param_.bdc_param);
  }
  processing_times_ms_["init_bdc_ptr"] = stopwatch_ms.toc("init_bdc_ptr");

  fmt::print("BoundaryDeparturePreventionModule::plan called.\n");

  stopwatch_ms.tic("plan_slow_down");
  auto result =
    plan_slow_down_intervals(raw_trajectory_points, smoothed_trajectory_points, planner_data);
  fmt::print("plan_slow_down takes {} ms.\n", stopwatch_ms.toc("plan_slow_down"));

  return result;
}

tl::expected<Output, std::string> BoundaryDeparturePreventionModule::plan(
  const PoseWithCovariance & pose_with_covariance, const TrajectoryPoints & ref_traj,
  const TrajectoryPoints & ego_pred_traj,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double ego_dist_on_traj,
  const VehicleInfo & vehicle_info)
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stopwatch_ms;
  std::map<const char *, double> processing_time_map;

  const auto abnormality_data_opt = boundary_departure_checker_ptr_->get_abnormalities_data(
    ego_pred_traj, aw_ref_traj, pose_with_covariance, *steering_angle_ptr_);

  if (!abnormality_data_opt) {
    return tl::make_unexpected(abnormality_data_opt.error());
  }

  output_.abnormalities_data = *abnormality_data_opt;

  stopwatch_ms.tic("get_closest_projected_to_boundaries");
  const auto closest_projections_to_bound_opt =
    boundary_departure_checker_ptr_->get_closest_projections_to_boundaries(
      aw_ref_traj, output_.abnormalities_data.projections_to_bound);
  processing_time_map["get_closest_projected_to_boundaries"] =
    stopwatch_ms.toc("get_closest_projected_to_boundaries");

  if (!closest_projections_to_bound_opt) {
    return tl::make_unexpected(closest_projections_to_bound_opt.error());
  }
  const auto & closest_projections_to_bound = *closest_projections_to_bound_opt;
  output_.closest_projections_to_bound = closest_projections_to_bound;

  stopwatch_ms.tic("get_departure_points");
  output_.departure_points = utils::get_departure_points(
    output_.closest_projections_to_bound, node_param_, vehicle_info, ego_dist_on_traj);
  processing_time_map["get_departure_points"] = stopwatch_ms.toc("get_departure_points");

  stopwatch_ms.tic("process_critical_departure");
  auto & crit_dpt_pts_mut = output_.critical_departure_points;
  for (auto & crit_dpt_pt_mut : crit_dpt_pts_mut) {
    crit_dpt_pt_mut.dist_on_traj =
      trajectory::closest(aw_ref_traj, crit_dpt_pt_mut.point_on_prev_traj);
    crit_dpt_pt_mut.dist_from_ego =
      crit_dpt_pt_mut.dist_on_traj - (ego_dist_on_traj + vehicle_info.max_longitudinal_offset_m);
    const auto updated_point = aw_ref_traj.compute(crit_dpt_pt_mut.dist_on_traj);
    crit_dpt_pt_mut.can_be_removed =
      autoware_utils::calc_distance2d(
        updated_point.pose.position, crit_dpt_pt_mut.point_on_prev_traj.pose.position) > 1.0;
  }
  processing_time_map["process_critical_departure"] =
    stopwatch_ms.toc("process_critical_departure");

  stopwatch_ms.tic("erase_critical_departure");
  utils::erase_if(crit_dpt_pts_mut, [](const DeparturePoint & pt) { return pt.can_be_removed; });
  processing_time_map["erase_critical_departure"] = stopwatch_ms.toc("erase_critical_departure");

  stopwatch_ms.tic("create_critical_departure_pts");
  for (const auto & dpt_pt : output_.departure_points) {
    if (dpt_pt.type == DepartureType::CRITICAL_DEPARTURE) {
      boundary_departure_checker::CriticalDeparturePoint crit_pt(dpt_pt);
      crit_pt.point_on_prev_traj = aw_ref_traj.compute(crit_pt.dist_on_traj);
      crit_dpt_pts_mut.push_back(crit_pt);
    }
  }
  processing_time_map["create_critical_departure_pts"] =
    stopwatch_ms.toc("create_critical_departure_pts");

  stopwatch_ms.tic("sort_critical_departure_pts");
  std::sort(crit_dpt_pts_mut.begin(), crit_dpt_pts_mut.end());
  processing_time_map["sort_critical_departure_pts"] =
    stopwatch_ms.toc("sort_critical_departure_pts");

  stopwatch_ms.tic("update_departure_interval");
  if (output_.departure_intervals.empty()) {
    output_.departure_intervals =
      utils::init_departure_intervals(aw_ref_traj, output_.departure_points, vehicle_info);
  } else {
    auto & departure_intervals_mut = output_.departure_intervals;
    auto & departure_points_mut = output_.departure_points;
    utils::update_departure_intervals(
      departure_intervals_mut, departure_points_mut, aw_ref_traj, vehicle_info, ref_traj.front(),
      ego_dist_on_traj);
  }
  processing_time_map["update_departure_interval"] = stopwatch_ms.toc("update_departure_interval");

  for (const auto & [name, time] : processing_time_map) {
    fmt::print("{}: {}\n", name, time);
  }

  return output_;
}

bool BoundaryDeparturePreventionModule::is_data_ready(
  [[maybe_unused]] std::unordered_map<std::string, double> & processing_times)
{
  if (!ego_pred_traj_ptr_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "waiting for predicted trajectory...");
    return false;
  }

  if (!op_mode_state_ptr_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "waiting for operation mode state...");
    return false;
  }

  return true;
}

bool BoundaryDeparturePreventionModule::is_data_valid() const
{
  if (ego_pred_traj_ptr_->points.empty()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "empty predicted trajectory...");
    return false;
  }
  return true;
}

bool BoundaryDeparturePreventionModule::is_data_timeout(const Odometry & odom) const
{
  const auto now = clock_ptr_->now();
  const auto time_diff_s = (rclcpp::Time(odom.header.stamp) - now).seconds();
  constexpr auto th_pose_timeout_s = 1.0;

  if (time_diff_s > th_pose_timeout_s) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_ptr_, throttle_duration_ms, "pose timeout...");
    return true;
  }

  return false;
}

bool BoundaryDeparturePreventionModule::found_nearby_points(
  const Point2d & candidate_point, const DeparturePoints & curr_departure_points)
{
  constexpr auto found_nearby{true};
  for (auto & point : curr_departure_points) {
    if (point.is_nearby(candidate_point)) {
      return found_nearby;
    }
  }

  return !found_nearby;
}

VelocityPlanningResult BoundaryDeparturePreventionModule::plan_slow_down_intervals(
  [[maybe_unused]] const TrajectoryPoints & raw_trajectory_points,
  [[maybe_unused]] const TrajectoryPoints & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & vehicle_info = planner_data->vehicle_info_;
  const auto & ll_map_ptr = planner_data->route_handler->getLaneletMapPtr();
  const auto & curr_odom = planner_data->current_odometry;
  const auto & curr_pose = curr_odom.pose;
  const auto & curr_position = curr_pose.pose.position;
  const auto & goal_position = raw_trajectory_points.back().pose.position;
  constexpr auto min_effective_dist = 1.0;

  StopWatch<std::chrono::milliseconds> stopwatch_ms;
  if (!ego_pred_traj_ptr_) {
    return {};
  }

  if (autoware_utils::calc_distance2d(curr_position, goal_position) < min_effective_dist) {
    output_.departure_intervals.clear();
    return {};
  }

  if (raw_trajectory_points.empty()) {
    return {};
  }

  stopwatch_ms.tic("convert_raw_traj_to_aw_traj");
  const auto aw_raw_traj_opt =
    trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(raw_trajectory_points);
  processing_times_ms_["convert_raw_traj_to_aw_traj"] =
    stopwatch_ms.toc("convert_raw_traj_to_aw_traj");

  if (!aw_raw_traj_opt) {
    return {};
  }

  const auto ego_dist_on_traj_m =
    experimental::trajectory::closest(*aw_raw_traj_opt, curr_pose.pose);

  stopwatch_ms.tic("find_slow_down_points");
  const auto output_opt = plan(
    curr_pose, raw_trajectory_points, ego_pred_traj_ptr_->points, *aw_raw_traj_opt,
    ego_dist_on_traj_m, vehicle_info);
  processing_times_ms_["find_slow_down_points"] = stopwatch_ms.toc("find_slow_down_points");

  if (!output_opt) {
    return {};
  }

  const auto & th_trigger = node_param_.bdc_param.th_trigger;
  SlowDownInterpolator interpolator(
    planner_data, 0.25, th_trigger.th_dist_to_boundary_m.left.max, th_trigger.max_slow_down_vel_mps,
    25.0 / 3.6, th_trigger.th_acc_mps2.min, th_trigger.th_acc_mps2.max);

  output_.is_critical_departing = std::any_of(
    output_.critical_departure_points.begin(), output_.critical_departure_points.end(),
    [&th_trigger](const DeparturePoint & pt) {
      const auto braking_dist = boundary_departure_checker::utils::compute_braking_distance(
        th_trigger.max_slow_down_vel_mps, 0.0, th_trigger.th_acc_mps2.min,
        th_trigger.th_jerk_mps3.max, th_trigger.brake_delay_s);
      return braking_dist >= pt.dist_from_ego;
    });

  updater_ptr_->force_update();

  output_.slow_down_intervals.clear();

  StopWatch<std::chrono::milliseconds> stopwatch;
  std::map<std::string, double> time_print;
  time_print["total"] = 0;
  std::vector<SlowdownInterval> slowdown_intervals;
  for (auto && pair : output_.departure_intervals | ranges::views::enumerate) {
    const auto & [idx, departure_interval] = pair;

    stopwatch.tic("lat_dist_to_bound_m");
    auto lat_dist_to_bound_m =
      output_.abnormalities_data
        .projections_to_bound[AbnormalityType::NORMAL][departure_interval.direction]
        .front()
        .lat_dist;
    time_print["lat_dist_to_bound_m"] = stopwatch.toc("lat_dist_to_bound_m");
    time_print["total"] += time_print["lat_dist_to_bound_m"];

    const auto dpt_pt_dist_on_traj_m =
      (ego_dist_on_traj_m < departure_interval.start_dist_on_traj)
        ? departure_interval.start_dist_on_traj
        : departure_interval.end_dist_on_traj - vehicle_info.max_longitudinal_offset_m;

    stopwatch.tic("interpolate_velocity");
    const auto vel_opt =
      interpolator.get_interp_to_point(dpt_pt_dist_on_traj_m, lat_dist_to_bound_m);
    time_print["interpolate_velocity"] = stopwatch.toc("interpolate_velocity");
    time_print["total"] += time_print["interpolate_velocity"];

    if (!vel_opt) {
      continue;
    }

    stopwatch.tic("dist_to_departure_point");
    const auto dist_to_departure_point =
      (departure_interval.start_dist_on_traj >
       (ego_dist_on_traj_m + vehicle_info.max_longitudinal_offset_m))
        ? departure_interval.start_dist_on_traj
        : departure_interval.end_dist_on_traj;

    time_print["dist_to_departure_point"] = stopwatch.toc("dist_to_departure_point");
    time_print["total"] += time_print["dist_to_departure_point"];

    stopwatch.tic("find_stop_pose");

    time_print["find_stop_pose"] = stopwatch.toc("find_stop_pose");
    time_print["total"] += time_print["find_stop_pose"];

    if (ego_dist_on_traj_m >= dist_to_departure_point) {
      continue;
    }

    const auto [rel_dist_m, vel, accel_mps2] = *vel_opt;

    auto end_pose = (departure_interval.start_dist_on_traj >
                     (ego_dist_on_traj_m + vehicle_info.max_longitudinal_offset_m))
                      ? departure_interval.start.pose.position
                      : departure_interval.end.pose.position;
    if (ego_dist_on_traj_m + std::get<0>(*vel_opt) > aw_raw_traj_opt->length()) {
      continue;
    }

    stopwatch.tic("find_start_pose");
    auto start_pose = aw_raw_traj_opt->compute(ego_dist_on_traj_m + rel_dist_m);
    time_print["find_start_pose"] = stopwatch.toc("find_start_pose");
    time_print["total"] += time_print["find_start_pose"];

    stopwatch.tic("inserting_point");
    output_.slow_down_intervals.emplace_back(start_pose.pose.position, end_pose);
    slowdown_intervals.emplace_back(start_pose.pose.position, end_pose, vel);
    time_print["inserting_point"] = stopwatch.toc("inserting_point");
    time_print["total"] += time_print["inserting_point"];
  }

  if (debug_publisher_) {
    slow_down_wall_marker_.markers.clear();
    autoware_utils::append_marker_array(
      debug::create_debug_marker_array(output_, clock_ptr_, curr_pose.pose.position.z, node_param_),
      &slow_down_wall_marker_);
    debug_publisher_->publish(slow_down_wall_marker_);
  }

  VelocityPlanningResult result;
  result.slowdown_intervals = slowdown_intervals;

  return result;
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::BoundaryDeparturePreventionModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
