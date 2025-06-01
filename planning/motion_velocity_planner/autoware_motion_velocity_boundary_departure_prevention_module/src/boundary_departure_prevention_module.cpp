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

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "debug.hpp"
#include "slow_down_interpolator.hpp"
#include "str_map.hpp"
#include "utils.hpp"

#include <autoware/boundary_departure_checker/utils.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <magic_enum.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{

void BoundaryDeparturePreventionModule::init(
  rclcpp::Node & node, [[maybe_unused]] const std::string & module_name)
{
  // fmt::print("Running Boundary Departure Prevention Module\n");
  module_name_ = module_name;
  clock_ptr_ = node.get_clock();
  logger_ = node.get_logger();

  node_param_ = param::NodeParam(node);
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

  auto boundary_behaviour_trigger_param = [&module_name,
                                           &parameters](const std::string & trigger_type_str) {
    BoundaryThreshold th_dist_to_boundary_m;
    const std::string ns{module_name + trigger_type_str + "."};
    const std::string ns_bound = ns + "th_dist_to_boundary_m.";
    update_param(parameters, ns_bound + "left", th_dist_to_boundary_m.left);
    update_param(parameters, ns_bound + "right", th_dist_to_boundary_m.right);

    param::BehaviorTriggerThreshold th_behavior;
    const std::string ns_behavior{ns + "th_trigger."};
    update_param(parameters, ns_behavior + "decel_mp2", th_behavior.decel_mp2);
    update_param(parameters, ns_behavior + "brake_delay_s", th_behavior.brake_delay_s);
    update_param(parameters, ns_behavior + "dist_error_m", th_behavior.dist_error_m);

    param::BoundaryBehaviorTrigger trigger;

    update_param(parameters, ns + "enable", trigger.enable);
    trigger.th_dist_to_boundary_m = th_dist_to_boundary_m;
    trigger.th_trigger = th_behavior;

    return trigger;
  };
  pp.slow_down_before_departure = boundary_behaviour_trigger_param("slow_down_before_departure");
  pp.stop_before_departure = boundary_behaviour_trigger_param("stop_before_departure");

  auto slow_down_behaviour_trigger_param = [&](
                                             const std::string & trigger_type_str,
                                             const param::SlowDownNearBoundaryTrigger slow) {
    param::SlowDownNearBoundaryTrigger trigger(boundary_behaviour_trigger_param(trigger_type_str));
    param::SlowDownBehavior slow_behavior;
    const std::string ns{module_name + trigger_type_str + "."};
    const std::string ns_slow = ns + "slow_down_behavior.";

    auto velocity_kmh = slow.slow_down_behavior.velocity_mps * 3.6;
    update_param(parameters, ns_slow + "velocity_kmh", velocity_kmh);
    slow_behavior.velocity_mps = velocity_kmh / 3.6;
    return trigger;
  };
  pp.slow_down_near_boundary =
    slow_down_behaviour_trigger_param("slow_down_near_boundary", pp.slow_down_near_boundary);

  const auto pred_path_fp_ns = module_name + "pred_path_footprint.";
  update_param(parameters, pred_path_fp_ns + "scale", pp.pred_path_footprint.scale);
  update_param(
    parameters, pred_path_fp_ns + "extra_margin_m", pp.pred_path_footprint.extra_margin_m);
  update_param(
    parameters, pred_path_fp_ns + "resample_interval_m",
    pp.pred_path_footprint.resample_interval_m);
}

void BoundaryDeparturePreventionModule::subscribe_topics(rclcpp::Node & node)
{
  // fmt::print("Subscribing\n");
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
  stopwatch_ms.tic(init_bdc_ptr);
  if (!boundary_departure_checker_ptr_) {
    boundary_departure_checker_ptr_ =
      std::make_unique<BoundaryDepartureChecker>(ll_map_ptr, vehicle_info, node_param_.bdc_param);
  }
  processing_times_ms_[init_bdc_ptr] = stopwatch_ms.toc(init_bdc_ptr);

  stopwatch_ms.tic("plan_slow_down");
  auto result =
    plan_slow_down_intervals(raw_trajectory_points, smoothed_trajectory_points, planner_data);
  fmt::print("plan_slow_down takes {} ms.\n", stopwatch_ms.toc("plan_slow_down"));

  return result;
}

tl::expected<param::Output, std::string> BoundaryDeparturePreventionModule::plan(
  const PoseWithCovariance & pose_with_covariance, const double abs_velocity,
  const TrajectoryPoints & ref_traj, const TrajectoryPoints & ego_pred_traj,
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double footprint_margin_scale,
  const double dist_to_ego, const VehicleInfo & vehicle_info)
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stopwatch_ms;
  const auto bdc_results =
    boundary_departure_checker_ptr_->get_projections_to_closest_uncrossable_boundaries(
      pose_with_covariance, abs_velocity, ego_pred_traj, aw_ref_traj, footprint_margin_scale,
      *steering_angle_ptr_);

  if (!bdc_results) {
    return tl::unexpected<std::string>(bdc_results.error());
  }

  stopwatch_ms.tic(convert_raw_traj_to_aw_traj);
  const auto aw_ego_pred_traj_opt =
    trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(ego_pred_traj_ptr_->points);
  processing_times_ms_[convert_raw_traj_to_aw_traj] = stopwatch_ms.toc(convert_raw_traj_to_aw_traj);

  if (!aw_ego_pred_traj_opt) {
    return tl::unexpected<std::string>(aw_ego_pred_traj_opt.error().what);
  }
  output_.aw_ego_traj = *aw_ego_pred_traj_opt;
  output_.aw_ref_traj = aw_ref_traj;
  output_.footprints = bdc_results->footprints;
  output_.boundary_segments = bdc_results->boundary_segments;
  output_.side_to_bound_projections = bdc_results->side_to_bound_projections;

  output_.processing_time_map["get_closest_boundary_segments_from_side;"] = stopwatch_ms.toc(true);

  for(const auto abnormality_key:abnormality_keys){
  output_.departure_statuses = utils::check_departure_status(
    output_.side_to_bound_projections[abnormality_key], node_param_,
    abs_velocity);

  }

  Side<DeparturePoints> dpts;
  DeparturePoints departure_points;
  for (const auto direction : side_keys) {
    auto & departure_statuses = output_.departure_statuses[direction];
    const auto & side_to_bound = output_.side_to_bound_projections["localization"][direction];

    for (const auto & [status, idx] : departure_statuses) {
      const auto & curr_side = side_to_bound[idx];

      DeparturePoint point;
      point.uuid = autoware_utils::to_hex_string(autoware_utils::generate_uuid());
      point.type = status;
      point.lat_dist_to_bound = curr_side.lat_dist;
      point.point = curr_side.pt_on_bound;
      point.direction = direction;
      point.th_dist_hysteresis = node_param_.th_dist_hysteresis_m;
      point.th_lat_dist_to_bounday_hyteresis = std::invoke([&]() -> double {
        if (point.type == DepartureType::CRITICAL_DEPARTURE) {
          return node_param_.stop_before_departure.th_dist_to_boundary_m[direction];
        }
        if (point.type == DepartureType::APPROACHING_DEPARTURE) {
          return node_param_.slow_down_before_departure.th_dist_to_boundary_m[direction];
        }

        if (point.type == DepartureType::NEAR_BOUNDARY) {
          return node_param_.slow_down_near_boundary.th_dist_to_boundary_m[direction];
        }

        return std::numeric_limits<double>::max();
      });

      point.dist_on_traj = trajectory::closest(output_.aw_ref_traj, point.to_geom_pt(0.0));
      point.dist_from_ego =
        point.dist_on_traj - (dist_to_ego + vehicle_info.max_longitudinal_offset_m);
      point.can_be_removed = point.dist_from_ego < std::numeric_limits<double>::epsilon();

      if (point.can_be_removed) {
        continue;
      }

      point.point_on_prev_traj = output_.aw_ref_traj.compute(point.dist_on_traj);
      dpts[point.direction].push_back(point);
    }
    std::sort(dpts[direction].begin(), dpts[direction].end());
    utils::erase_after_first_match(dpts[direction]);
    std::move(dpts[direction].begin(), dpts[direction].end(), std::back_inserter(departure_points));
  }

  auto & departure_intervals_mut = output_.departure_intervals;
  if (departure_intervals_mut.empty()) {
    size_t idx = 0;
    while (idx < departure_points.size()) {
      DepartureInterval interval;
      interval.start = departure_points[idx].point_on_prev_traj;
      interval.start_dist_on_traj = departure_points[idx].dist_on_traj;
      interval.candidates.push_back(departure_points[idx]);
      interval.direction = departure_points[idx].direction;

      size_t idx_end = idx + 1;
      while (idx_end < departure_points.size() &&
             departure_points[idx_end].direction == interval.direction &&
             departure_points[idx_end].type == DepartureType::NEAR_BOUNDARY) {
        const auto & prev = departure_points[idx_end - 1];
        const auto & curr = departure_points[idx_end];
        const auto diff = std::abs(curr.dist_on_traj - prev.dist_on_traj);

        if (diff < vehicle_info.max_longitudinal_offset_m && curr.direction == prev.direction) {
          interval.candidates.push_back(curr);
          ++idx_end;
        } else {
          break;
        }
      }

      interval.end = interval.candidates.back().point_on_prev_traj;
      interval.end_dist_on_traj = interval.candidates.back().dist_on_traj;
      departure_intervals_mut.push_back(interval);
      idx = idx_end;
    }
  } else {
    // check if path is shifted
    for (auto & departure_interval_mut : departure_intervals_mut) {
      // update start end pose
      if (!departure_interval_mut.start_at_traj_front) {
        departure_interval_mut.start_dist_on_traj =
          trajectory::closest(aw_ref_traj, departure_interval_mut.start);
        constexpr auto th_dist_from_start{1.0};
        departure_interval_mut.start_at_traj_front =
          departure_interval_mut.start_dist_on_traj < th_dist_from_start;
      } else {
        departure_interval_mut.start_dist_on_traj = 0.0;
        departure_interval_mut.start = ref_traj.front();
      }
      departure_interval_mut.end_dist_on_traj =
        trajectory::closest(aw_ref_traj, departure_interval_mut.end);
    }

    // remove if ego already pass the end pose.
    departure_intervals_mut.erase(
      std::remove_if(
        departure_intervals_mut.begin(), departure_intervals_mut.end(),
        [&](const DepartureInterval & interval) {
          if (interval.end_dist_on_traj < dist_to_ego) {
            return true;
          }
          auto point_of_curr_traj = aw_ref_traj.compute(interval.end_dist_on_traj);
          return autoware_utils::calc_distance2d(
                   interval.end.pose.position, point_of_curr_traj.pose.position) > 0.25;
        }),
      departure_intervals_mut.end());

    // check if departure point is in between any intervals.
    // if close to end pose, update end pose.
    for (auto & departure_interval_mut : departure_intervals_mut) {
      for (auto & departure_point_mut : departure_points) {
        if (departure_point_mut.can_be_removed) {
          continue;
        }

        if (
          departure_interval_mut.end_dist_on_traj <
            departure_point_mut.dist_on_traj + vehicle_info.max_longitudinal_offset_m &&
          departure_interval_mut.direction == departure_point_mut.direction) {
          // althought name is point on prev traj, we already update them earlier
          departure_interval_mut.end = departure_point_mut.point_on_prev_traj;
          departure_point_mut.can_be_removed = true;
        }
      }
    }
  }
  output_.departure_points = std::move(departure_points);

  fmt::print("total num of intervals {}\n", output_.departure_intervals.size());

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
  const auto & odom_abs_vel = std::abs(curr_odom.twist.twist.linear.x);
  constexpr auto min_vel = 0.01;
  constexpr auto min_effective_dist = 1.0;
  const auto abs_vel = odom_abs_vel < min_vel ? 0.0 : odom_abs_vel;

  StopWatch<std::chrono::milliseconds> stopwatch_ms;
  if (!ego_pred_traj_ptr_) {
    fmt::print("Ego predicted trajectory is invalid.\n");
    return {};
  }

  if (autoware_utils::calc_distance2d(curr_position, goal_position) < min_effective_dist) {
    fmt::print("Distance from ego to goal is less than {}.\n", min_effective_dist);
    output_.departure_intervals.clear();
    return {};
  }

  if (raw_trajectory_points.empty()) {
    fmt::print("raw_trajectory_points is empty.\n");
    return {};
  }

  stopwatch_ms.tic(convert_raw_traj_to_aw_traj);
  const auto aw_raw_traj_opt =
    trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(raw_trajectory_points);
  processing_times_ms_[convert_raw_traj_to_aw_traj] = stopwatch_ms.toc(convert_raw_traj_to_aw_traj);

  if (!aw_raw_traj_opt) {
    fmt::print(
      "Failed to convert raw_trajectory_points. Reason: {}\n", aw_raw_traj_opt.error().what);
    return {};
  }

  const auto dist_to_ego =
    experimental::trajectory::closest(*aw_raw_traj_opt, ego_pred_traj_ptr_->points.front().pose);

  stopwatch_ms.tic(find_slow_down_points);
  const auto output_opt = plan(
    curr_pose, abs_vel, raw_trajectory_points, ego_pred_traj_ptr_->points, *aw_raw_traj_opt,
    node_param_.pred_path_footprint.scale, dist_to_ego, vehicle_info);
  processing_times_ms_[find_slow_down_points] = stopwatch_ms.toc(find_slow_down_points);

  if (!output_opt) {
    fmt::print("failed reason({}): {}", __func__, output_opt.error());
    return {};
  }

  SlowDownInterpolator interpolator(planner_data, 0.25, 0.5, 5.0 / 3.6, 35.0 / 3.6, -1.5);

  output_.is_critical_departing = std::any_of(
    output_.departure_points.begin(), output_.departure_points.end(),
    [abs_vel](const DeparturePoint & pt) {
      const auto braking_dist = utils::compute_braking_distance(abs_vel, 0.0, -2.5, -1.5);
      fmt::print("braking_dist = {} [m]\n", braking_dist);
      return braking_dist < pt.dist_from_ego && pt.type == DepartureType::CRITICAL_DEPARTURE;
    });

  if (output_.is_critical_departing) {
    fmt::print("is critical departure\n");
  }

  updater_ptr_->force_update();

  output_.slow_down_interval.clear();

  StopWatch<std::chrono::milliseconds> stopwatch;
  std::map<std::string, double> time_print;
  time_print["total"] = 0;
  std::vector<SlowdownInterval> slowdown_intervals;
  for (auto && pair : output_.departure_intervals | ranges::views::enumerate) {
    const auto & [idx, departure_interval] = pair;

    stopwatch.tic("lat_dist_to_bound_m");
    auto lat_dist_to_bound_m =
      output_.side_to_bound_projections["localization"][departure_interval.direction].front().lat_dist;
    time_print["lat_dist_to_bound_m"] = stopwatch.toc("lat_dist_to_bound_m");
    time_print["total"] += time_print["lat_dist_to_bound_m"];

    stopwatch.tic("dpt_pt_dist_on_traj_m");
    const auto dpt_pt_dist_on_traj_m = departure_interval.start_dist_on_traj;
    time_print["dpt_pt_dist_on_traj_m"] = stopwatch.toc("dpt_pt_dist_on_traj_m");
    time_print["total"] += time_print["dpt_pt_dist_on_traj_m"];

    stopwatch.tic("interpolate_velocity");
    const auto vel_opt =
      interpolator.get_interp_to_point(dpt_pt_dist_on_traj_m, lat_dist_to_bound_m);
    time_print["interpolate_velocity"] = stopwatch.toc("interpolate_velocity");
    time_print["total"] += time_print["interpolate_velocity"];

    if (!vel_opt) {
      fmt::print("{}\n", vel_opt.error());

      for (const auto & [key, duration] : time_print) {
        if (key == "total") {
          continue;
        }
        fmt::print("idx: {}, key: {}, time: {}\n", idx, key, duration);
      }
      continue;
    }

    stopwatch.tic("dist_to_departure_point");
    const auto dist_to_departure_point = (departure_interval.start_dist_on_traj >
                                          (dist_to_ego + vehicle_info.max_longitudinal_offset_m))
                                           ? departure_interval.start_dist_on_traj
                                           : departure_interval.end_dist_on_traj;

    time_print["dist_to_departure_point"] = stopwatch.toc("dist_to_departure_point");
    time_print["total"] += time_print["dist_to_departure_point"];

    stopwatch.tic("find_stop_pose");

    time_print["find_stop_pose"] = stopwatch.toc("find_stop_pose");
    time_print["total"] += time_print["find_stop_pose"];

    if (dist_to_ego >= dist_to_departure_point) {
      fmt::print(
        "Slow down distance exceeded distance to slow down point by {} [m]\n",
        dist_to_ego - dist_to_departure_point);

      for (const auto & [key, duration] : time_print) {
        if (key == "total") {
          continue;
        }
        fmt::print("idx: {}, key: {}, time: {}\n", idx, key, duration);
      }
      continue;
    }

    const auto vel = std::get<1>(*vel_opt);

    auto end_pose = (departure_interval.start_dist_on_traj >
                     (dist_to_ego + vehicle_info.max_longitudinal_offset_m))
                      ? departure_interval.start.pose.position
                      : departure_interval.end.pose.position;
    if (dist_to_ego + std::get<0>(*vel_opt) > aw_raw_traj_opt->length()) {
      continue;
    }

    stopwatch.tic("find_start_pose");
    auto start_pose = aw_raw_traj_opt->compute(dist_to_ego + std::get<0>(*vel_opt));
    time_print["find_start_pose"] = stopwatch.toc("find_start_pose");
    time_print["total"] += time_print["find_start_pose"];

    stopwatch.tic("inserting_point");
    output_.slow_down_interval.emplace_back(start_pose.pose.position, end_pose);
    slowdown_intervals.emplace_back(start_pose.pose.position, end_pose, vel);
    time_print["inserting_point"] = stopwatch.toc("inserting_point");
    time_print["total"] += time_print["inserting_point"];

    for (const auto & [key, duration] : time_print) {
      if (key == "total") {
        continue;
      }
      fmt::print("idx: {}, key: {}, time: {}\n", idx, key, duration);
    }
  }

  fmt::print("Total time {} ms\n", time_print["total"]);

  if (debug_publisher_) {
    slow_down_wall_marker_.markers.clear();
    autoware_utils::append_marker_array(
      debug::create_debug_marker_array(output_, clock_ptr_, curr_pose.pose.position.z),
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
