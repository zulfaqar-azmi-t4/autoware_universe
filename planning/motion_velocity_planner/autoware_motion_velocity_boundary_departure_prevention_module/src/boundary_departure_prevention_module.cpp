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
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <magic_enum.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view.hpp>

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
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(processing_time_detail_pub_);
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
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto ns = get_module_name();

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns + "/debug_markers", 1);

  virtual_wall_publisher_ = node.create_publisher<MarkerArray>("~/" + ns + "/virtual_walls", 1);

  processing_time_detail_pub_ = node.create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/" + ns, 1);

  processing_time_publisher_ =
    node.create_publisher<Float64Stamped>("~/debug/obstacle_slow_down/processing_time_ms", 1);
}

VelocityPlanningResult BoundaryDeparturePreventionModule::plan(
  const TrajectoryPoints & raw_trajectory_points,
  [[maybe_unused]] const TrajectoryPoints & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  StopWatch<std::chrono::milliseconds> stopwatch_ms;
  stopwatch_ms.tic("plan");

  const auto & vehicle_info = planner_data->vehicle_info_;
  const auto & ll_map_ptr = planner_data->route_handler->getLaneletMapPtr();

  if (!boundary_departure_checker_ptr_) {
    boundary_departure_checker_ptr_ =
      std::make_unique<BoundaryDepartureChecker>(ll_map_ptr, vehicle_info, node_param_.bdc_param);
  }

  if (!slow_down_interpolator_ptr_) {
    slow_down_interpolator_ptr_ =
      std::make_unique<SlowDownInterpolator>(planner_data, node_param_.bdc_param.th_trigger);
  }

  auto result_opt = plan_slow_down_intervals(raw_trajectory_points, planner_data);

  if (!result_opt) {
    RCLCPP_DEBUG(rclcpp::get_logger(get_module_name()), "%s", result_opt.error().c_str());
  }

  processing_time_publisher_->publish(std::invoke([&]() {
    autoware_internal_debug_msgs::msg::Float64Stamped msg;
    msg.stamp = clock_ptr_->now();
    msg.data = stopwatch_ms.toc();
    return msg;
  }));

  return *result_opt;
}

bool BoundaryDeparturePreventionModule::is_data_ready(
  [[maybe_unused]] std::unordered_map<std::string, double> & processing_times)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (ego_pred_traj_ptr_->points.empty()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "empty predicted trajectory...");
    return false;
  }
  return true;
}

bool BoundaryDeparturePreventionModule::is_data_timeout(const Odometry & odom) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto now = clock_ptr_->now();
  const auto time_diff_s = (rclcpp::Time(odom.header.stamp) - now).seconds();
  constexpr auto th_pose_timeout_s = 1.0;

  if (time_diff_s > th_pose_timeout_s) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_ptr_, throttle_duration_ms, "pose timeout...");
    return true;
  }

  return false;
}

tl::expected<VelocityPlanningResult, std::string>
BoundaryDeparturePreventionModule::plan_slow_down_intervals(
  const TrajectoryPoints & raw_trajectory_points,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & vehicle_info = planner_data->vehicle_info_;
  const auto & ll_map_ptr = planner_data->route_handler->getLaneletMapPtr();
  const auto & curr_odom = planner_data->current_odometry;
  const auto & curr_pose = curr_odom.pose;
  const auto & curr_position = curr_pose.pose.position;
  const auto & goal_position = raw_trajectory_points.back().pose.position;
  constexpr auto min_effective_dist = 1.0;

  StopWatch<std::chrono::milliseconds> stopwatch_ms;
  const auto toc_processing_time = [&](std::string && tag) {
    processing_times_ms_[tag] = stopwatch_ms.toc(tag);
  };

  const auto toc_processing_time_sum = [&](std::string && tag) {
    if (processing_times_ms_.find(tag) == processing_times_ms_.end()) {
      processing_times_ms_[tag] = stopwatch_ms.toc(tag);
      return;
    }

    processing_times_ms_[tag] += stopwatch_ms.toc(tag);
  };

  if (!ego_pred_traj_ptr_) {
    return tl::make_unexpected("Couldn't read predicted path pointer.");
  }

  if (raw_trajectory_points.empty()) {
    return tl::make_unexpected("Empty reference trajectory.");
  }
  if (autoware_utils::calc_distance2d(curr_position, goal_position) < min_effective_dist) {
    output_.departure_intervals.clear();
    return tl::make_unexpected("Too close to goal.");
  }
  toc_processing_time("chk_dist_to_goal");

  const auto ref_traj_pts_opt =
    trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(raw_trajectory_points);
  toc_processing_time("build_aw_raw_traj_opt");

  if (!ref_traj_pts_opt) {
    return tl::make_unexpected(ref_traj_pts_opt.error().what);
  }

  const auto abnormality_data_opt = boundary_departure_checker_ptr_->get_abnormalities_data(
    ego_pred_traj_ptr_->points, *ref_traj_pts_opt, curr_pose, *steering_angle_ptr_);
  toc_processing_time("get_abnormalities_data");

  if (!abnormality_data_opt) {
    return tl::make_unexpected(abnormality_data_opt.error());
  }

  output_.abnormalities_data = *abnormality_data_opt;

  const auto closest_projections_to_bound_opt =
    boundary_departure_checker_ptr_->get_closest_projections_to_boundaries(
      *ref_traj_pts_opt, output_.abnormalities_data.projections_to_bound);
  toc_processing_time("closest_projection_to_boundaries");

  if (!closest_projections_to_bound_opt) {
    return tl::make_unexpected(closest_projections_to_bound_opt.error());
  }

  output_.closest_projections_to_bound = *closest_projections_to_bound_opt;

  const auto ego_dist_on_traj_m =
    experimental::trajectory::closest(*ref_traj_pts_opt, curr_pose.pose);
  toc_processing_time("get_ego_dist_on_traj_m");

  const auto ego_dist_on_traj_with_offset_m = [&](const bool take_front_offset) {
    const auto offset_m = take_front_offset ? vehicle_info.max_longitudinal_offset_m
                                            : vehicle_info.min_longitudinal_offset_m;
    return ego_dist_on_traj_m + offset_m;
  };

  output_.departure_points =
    boundary_departure_checker_ptr_->get_departure_points(output_.closest_projections_to_bound);
  toc_processing_time("get_departure_points");

  utils::update_critical_departure_points(
    output_.departure_points, output_.critical_departure_points, *ref_traj_pts_opt,
    node_param_.bdc_param.th_dist_hysteresis_m,
    ego_dist_on_traj_with_offset_m(!planner_data->is_driving_forward));
  toc_processing_time("update_critical_departure_points");

  if (output_.departure_intervals.empty()) {
    output_.departure_intervals = utils::init_departure_intervals(
      *ref_traj_pts_opt, output_.departure_points,
      ego_dist_on_traj_with_offset_m(!planner_data->is_driving_forward));
  } else {
    auto & departure_intervals_mut = output_.departure_intervals;
    const auto & ref_traj_front_pt = raw_trajectory_points.front();
    utils::update_departure_intervals(
      departure_intervals_mut, output_.departure_points, *ref_traj_pts_opt,
      vehicle_info.vehicle_length_m, ref_traj_front_pt,
      ego_dist_on_traj_with_offset_m(!planner_data->is_driving_forward));
  }
  toc_processing_time("update_departure_interval");

  const auto & th_trigger = node_param_.bdc_param.th_trigger;
  output_.is_critical_departing = std::any_of(
    output_.critical_departure_points.begin(), output_.critical_departure_points.end(),
    [&th_trigger, &planner_data, &ego_dist_on_traj_with_offset_m](const DeparturePoint & pt) {
      const auto braking_dist = boundary_departure_checker::utils::compute_braking_distance(
        th_trigger.max_slow_down_vel_mps, 0.0, th_trigger.th_acc_mps2.min,
        th_trigger.th_jerk_mps3.max, th_trigger.brake_delay_s);
      return pt.dist_on_traj - ego_dist_on_traj_with_offset_m(planner_data->is_driving_forward) <=
             braking_dist;
    });

  updater_ptr_->force_update();

  output_.slow_down_intervals.clear();
  slow_down_wall_marker_.markers.clear();
  std::vector<SlowdownInterval> slowdown_intervals;

  for (auto && pair : output_.departure_intervals | ranges::views::enumerate) {
    const auto & [idx, departure_interval] = pair;

    const auto dpt_pt_dist_on_traj_m = (ego_dist_on_traj_m < departure_interval.start_dist_on_traj)
                                         ? departure_interval.start.pose
                                         : departure_interval.end.pose;

    auto lat_dist_to_bound_m =
      boundary_departure_checker::utils::get_nearest_boundary_segment_from_point(
        output_.abnormalities_data.boundary_segments[departure_interval.side_key],
        utils::to_pt2d(dpt_pt_dist_on_traj_m.position));
    toc_processing_time_sum("get_nearest_boundary_segment_from_point");

    if (!lat_dist_to_bound_m) {
      continue;
    }

    const auto vel_opt = slow_down_interpolator_ptr_->get_interp_to_point(
      dpt_pt_dist_on_traj_m.position, *lat_dist_to_bound_m, departure_interval.side_key);
    toc_processing_time("interpolate_slow_down_velocity");

    if (!vel_opt) {
      continue;
    }

    const auto dist_to_departure_point =
      (departure_interval.start_dist_on_traj >
       (ego_dist_on_traj_m + vehicle_info.max_longitudinal_offset_m))
        ? departure_interval.start_dist_on_traj
        : departure_interval.end_dist_on_traj;

    if (ego_dist_on_traj_m >= dist_to_departure_point) {
      continue;
    }

    const auto [rel_dist_m, vel, accel_mps2] = *vel_opt;

    auto end_pose = (departure_interval.start_dist_on_traj >
                     (ego_dist_on_traj_m + vehicle_info.max_longitudinal_offset_m))
                      ? departure_interval.start.pose
                      : departure_interval.end.pose;
    if (ego_dist_on_traj_m + std::get<0>(*vel_opt) > ref_traj_pts_opt->length()) {
      continue;
    }

    auto start_pose = ref_traj_pts_opt->compute(ego_dist_on_traj_m + rel_dist_m);
    toc_processing_time_sum("compute_slow_down_start_pose");

    output_.slow_down_intervals.emplace_back(start_pose.pose.position, end_pose.position);
    slowdown_intervals.emplace_back(start_pose.pose.position, end_pose.position, vel);
    toc_processing_time_sum("insert_slow_down_intervals");

    const auto markers_start = autoware::motion_utils::createSlowDownVirtualWallMarker(
      start_pose.pose, "boundary_departure_prevention_start", clock_ptr_->now(),
      static_cast<int32_t>(idx), vehicle_info.max_longitudinal_offset_m, "",
      planner_data->is_driving_forward);
    autoware_utils::append_marker_array(markers_start, &slow_down_wall_marker_);
    const auto markers_end = autoware::motion_utils::createSlowDownVirtualWallMarker(
      end_pose, "boundary_departure_prevention_end", clock_ptr_->now(),
      static_cast<int32_t>(idx + output_.departure_intervals.size() + 1),
      vehicle_info.max_longitudinal_offset_m, "", planner_data->is_driving_forward);
    autoware_utils::append_marker_array(markers_end, &slow_down_wall_marker_);
    toc_processing_time_sum("insert_slow_down_markers");
  }
  if (virtual_wall_publisher_) {
    virtual_wall_publisher_->publish(slow_down_wall_marker_);
  }
  toc_processing_time("publish_slow_down_marker");

  if (debug_publisher_) {
    debug_marker_.markers.clear();
    autoware_utils::append_marker_array(
      debug::create_debug_marker_array(output_, clock_ptr_, curr_pose.pose.position.z, node_param_),
      &debug_marker_);
    debug_publisher_->publish(debug_marker_);
  }
  toc_processing_time("publish_debug_marker");

  VelocityPlanningResult result;
  result.slowdown_intervals = slowdown_intervals;

  return result;
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::BoundaryDeparturePreventionModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
