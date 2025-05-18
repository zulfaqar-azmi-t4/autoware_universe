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
#include "str_map.hpp"
#include "utils.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/boundary_departure_checker/utils.hpp>
#include <magic_enum.hpp>

#include <fmt/format.h>

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
  StopWatch<std::chrono::milliseconds> stopwatch_ms;
  const auto & vehicle_info = planner_data->vehicle_info_;
  const auto & ll_map_ptr = planner_data->route_handler->getLaneletMapPtr();

  if (!boundary_departure_checker_ptr_) {
    stopwatch_ms.tic(init_bdc_ptr);
    boundary_departure_checker_ptr_ =
      std::make_unique<BoundaryDepartureChecker>(ll_map_ptr, vehicle_info, node_param_.bdc_param);
    processing_times_ms_[init_bdc_ptr] = stopwatch_ms.toc(init_bdc_ptr);
  }

  const auto & curr_odom = planner_data->current_odometry;
  const auto & curr_pose = curr_odom.pose;
  if (raw_trajectory_points.size() > 1) {
    [[maybe_unused]]auto trajectory = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(raw_trajectory_points);
    // const auto temp = experimental::trajectory::closest(*trajectory, ego_pred_traj_ptr_->points.front().pose);

    auto idx = 0;
    for(const auto dist : trajectory->get_underlying_bases()){
      fmt::print("{}: {}\n", ++idx, dist);
    }
  }

  if (!ego_pred_traj_ptr_ || raw_trajectory_points.empty()) {
    return {};
  }

  const auto & curr_twist = curr_odom.twist.twist;
  const auto & goal_pose = raw_trajectory_points.back().pose;

  if(autoware_utils::calc_distance2d(curr_pose.pose.position, goal_pose.position) < 1.0){
    return {};
  }

  constexpr double min_velocity = 0.01;
  const auto raw_abs_velocity = std::abs(curr_twist.linear.x);
  const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;

  stopwatch_ms.tic(find_slow_down_points);
  const auto output_opt = plan(
    curr_pose, abs_velocity, ego_pred_traj_ptr_->points, node_param_.pred_path_footprint.scale);
  if (!output_opt) {
    fmt::print("failed reason({}): {}", __func__, output_opt.error());
    return {};
  }
  processing_times_ms_[find_slow_down_points] = stopwatch_ms.toc(find_slow_down_points);

  if (debug_publisher_) {
    slow_down_wall_marker_.markers.clear();
    autoware_utils::append_marker_array(
      debug::create_debug_marker_array(output_, clock_ptr_, curr_pose.pose.position.z),
      &slow_down_wall_marker_);
    debug_publisher_->publish(slow_down_wall_marker_);
  }

  auto slow_down_traj_points = raw_trajectory_points;

  const auto dist_to_ego = std::invoke([&]() {
    const auto idx = planner_data->find_segment_index(raw_trajectory_points, curr_pose.pose);
    using autoware::motion_utils::calcSignedArcLength;
    return calcSignedArcLength(slow_down_traj_points, 0, curr_pose.pose.position, idx);
  });

  const auto insert_to_traj = [&](const double lon_dist) -> std::optional<size_t> {
    using autoware::motion_utils::insertTargetPoint;
    if (const auto inserted_idx_opt = insertTargetPoint(0, lon_dist, slow_down_traj_points)) {
      if (*inserted_idx_opt + 2 <= slow_down_traj_points.size()) {
        slow_down_traj_points.at(*inserted_idx_opt).longitudinal_velocity_mps =
          slow_down_traj_points.at(*inserted_idx_opt + 1).longitudinal_velocity_mps;
      }
      return *inserted_idx_opt;
    }
    return std::nullopt;
  };

  std::vector<std::pair<size_t, size_t>> slow_down_candidate_idx;
  // for (const auto & [uuid, departure_point] : output_.departure_points) {
  //   const auto & status = departure_point.type;
  //   if(status != DepartureType::CRITICAL_DEPARTURE){
  //     continue;
  //   }

  //   auto slow_down_candidates = utils::get_traj_indices_candidates(
  //     status, output_.ego_sides_from_footprints, vehicle_info.vehicle_length_m);
  //   std::move(
  //     slow_down_candidates.begin(), slow_down_candidates.end(),
  //     std::back_inserter(slow_down_candidate_idx));
  // }
  output_.is_critical_departing = false;

  updater_ptr_->force_update();

  return {};
  std::vector<SlowdownInterval> slowdown_intervals;
  int i = 0;
  for (const auto & [start_idx, end_idx] : slow_down_candidate_idx) {
    if (
      start_idx >= output_.ego_sides_from_footprints.size() ||
      end_idx >= output_.ego_sides_from_footprints.size()) {
      continue;
    }
    const auto start_dist = output_.ego_sides_from_footprints.at(start_idx).dist_from_start;
    const auto end_dist = output_.ego_sides_from_footprints.at(end_idx).dist_from_start;

    const auto slow_down_start_idx = insert_to_traj(start_dist + dist_to_ego);
    const auto slow_down_end_idx =
      (start_dist < end_dist) ? insert_to_traj(end_dist + dist_to_ego) : std::nullopt;

    if (!slow_down_start_idx) {
      continue;
    }

    slowdown_intervals.emplace_back(
      slow_down_traj_points.at(slow_down_start_idx ? *slow_down_start_idx : 0).pose.position,
      slow_down_traj_points.at(*slow_down_end_idx).pose.position,
      node_param_.slow_down_near_boundary.slow_down_behavior.velocity_mps);

    if (slow_down_start_idx && slow_down_end_idx) {
      const auto slow_down_wall_idx = std::invoke([&]() {
        const auto ego_idx = planner_data->find_index(slow_down_traj_points, curr_pose.pose);
        if (ego_idx < *slow_down_start_idx) return *slow_down_start_idx;
        if (ego_idx < *slow_down_end_idx) return ego_idx;
        return *slow_down_end_idx;
      });

      slow_down_wall_marker_.markers.clear();
      const auto marker = autoware::motion_utils::createSlowDownVirtualWallMarker(
        slow_down_traj_points.at(slow_down_wall_idx).pose, "boundary departure", clock_ptr_->now(),
        ++i);
      autoware_utils::append_marker_array(marker, &slow_down_wall_marker_);
    }
  }
  VelocityPlanningResult result;
  result.slowdown_intervals = slowdown_intervals;

  return {};
}

tl::expected<param::Output, std::string> BoundaryDeparturePreventionModule::plan(
  const PoseWithCovariance & pose_with_covariance, const double abs_velocity,
  const TrajectoryPoints & ego_pred_traj, const double footprint_margin_scale)
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  const auto bdc_results =
    boundary_departure_checker_ptr_->get_projections_to_closest_uncrossable_boundaries(
      pose_with_covariance, abs_velocity, ego_pred_traj, footprint_margin_scale);

  if (!bdc_results) {
    fmt::print("Failed reason({}): {}.\n", __func__, bdc_results.error());
    return tl::unexpected<std::string>(bdc_results.error());
  }

  output_.ab_enveloped_fp = bdc_results->ab_enveloped_fp;
  output_.ab_lon_tracking_fp = bdc_results->ab_lon_tracking_fp;
  output_.ego_sides_from_footprints = bdc_results->ego_sides_from_footprints;
  output_.boundary_segments = bdc_results->boundary_segments;
  output_.side_to_bound_projections = bdc_results->side_to_bound_projections;
  output_.ab_steering_fp = bdc_results->ab_steering_fp;

  output_.processing_time_map["get_closest_boundary_segments_from_side;"] = stop_watch.toc(true);

  output_.departure_statuses = utils::check_departure_status(
    output_.ego_sides_from_footprints, output_.side_to_bound_projections, node_param_,
    abs_velocity);

  check_departure_points_lifetime();
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
  const Point2d & candidate_point, DeparturePoints & curr_departure_points)
{
  constexpr auto found_nearby{true};
  for (auto & [uuid, point] : curr_departure_points) {
    if (point.is_nearby(candidate_point)) {
      point.lifetime = departure_points_lifetime_watch_.toc(uuid, true);
      return found_nearby;
    }
  }

  return !found_nearby;
}

void BoundaryDeparturePreventionModule::remove_exist_point(
  [[maybe_unused]] const std::vector<ProjectionWithSegment> & projections,
  [[maybe_unused]] DeparturePoints & departure_points,
  std::vector<param::DepartureTypeIdx> & statuses)
{
  [[maybe_unused]] auto & departures = statuses;

  if (departures.empty()) {
    for (auto & [uuid, point] : departure_points) {
      point.lifetime = departure_points_lifetime_watch_.toc(uuid);
    }
  }

  auto remove_itr =
    std::remove_if(departures.begin(), departures.end(), [&](const auto & departure) {
      const auto & [status, idx] = departure;
      auto side_to_bound_proj = projections[idx];
      auto proj = side_to_bound_proj.projection.proj;
      return found_nearby_points(proj, departure_points);
    });

  statuses.erase(remove_itr, statuses.end());
}

void BoundaryDeparturePreventionModule::check_departure_points_lifetime()
{
  const auto & side_to_bound = output_.side_to_bound_projections;
  auto & departure_statuses = output_.departure_statuses;

  for (const auto side_key : side_keys) {
    remove_exist_point(
      side_to_bound[side_key], output_.departure_points, departure_statuses[side_key]);
    auto & departure_points = output_.departure_points;
    for (auto it = departure_points.begin(); it != departure_points.end();) {
      if (!it->second.is_alive()) {
        it = departure_points.erase(it);
      } else {
        ++it;
      }
    }
    for (const auto & [status, idx] : output_.departure_statuses[side_key]) {
      const auto & proj = side_to_bound[side_key][idx].projection.proj;
      auto departure_point = utils::create_departure_point(proj, status, node_param_);
      output_.departure_points.insert(departure_point);
      departure_points_lifetime_watch_.tic(departure_point.first);
    }
  }
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::BoundaryDeparturePreventionModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
