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

#include "manual_trajectory_helper.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::manual_trajectory::helper
{

namespace
{
/**
 * @brief Interpolates a value from the velocity profile.
 */
std::optional<VelocityPoint> interp_profile(const std::vector<VelocityPoint> & profile, double s)
{
  if (profile.empty()) return std::nullopt;
  auto search_fn = [](const VelocityPoint & p, double val) { return p.s < val; };
  auto it = std::lower_bound(profile.begin(), profile.end(), s, search_fn);
  size_t idx = std::distance(profile.begin(), it);
  size_t prev = (idx == 0) ? 0 : idx - 1;
  size_t next = (idx >= profile.size()) ? profile.size() - 1 : idx;
  double ds = profile[next].s - profile[prev].s;
  double ratio = (ds > g_epsilon) ? std::clamp((s - profile[prev].s) / ds, 0.0, 1.0) : 0.0;
  VelocityPoint res;
  res.s = s;
  res.v = profile[prev].v + ratio * (profile[next].v - profile[prev].v);
  res.a = profile[prev].a + ratio * (profile[next].a - profile[prev].a);
  return res;
}

std::optional<geometry_msgs::msg::Pose> interp_geometry(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path,
  const std::vector<double> & path_s, double s)
{
  if (path.empty()) return std::nullopt;
  auto it = std::lower_bound(path_s.begin(), path_s.end(), s);
  size_t idx = std::distance(path_s.begin(), it);
  size_t prev = (idx == 0) ? 0 : idx - 1;
  size_t next = (idx >= path.size()) ? path.size() - 1 : idx;
  double ds = path_s[next] - path_s[prev];
  double ratio = (ds > g_epsilon) ? std::clamp((s - path_s[prev]) / ds, 0.0, 1.0) : 0.0;
  const auto & p0 = path[prev].point.pose;
  const auto & p1 = path[next].point.pose;
  geometry_msgs::msg::Pose pose;
  pose.position.x = p0.position.x + ratio * (p1.position.x - p0.position.x);
  pose.position.y = p0.position.y + ratio * (p1.position.y - p0.position.y);
  pose.position.z = p0.position.z + ratio * (p1.position.z - p0.position.z);
  tf2::Quaternion q0, q1;
  tf2::fromMsg(p0.orientation, q0);
  tf2::fromMsg(p1.orientation, q1);
  pose.orientation = tf2::toMsg(q0.slerp(q1, ratio));
  return pose;
}

// Updated: Add min_decel to apply analytical limit
SpatialProfile create_spatial_profile(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path,
  const double default_limit, const double stop_dist_limit, const double min_decel)
{
  SpatialProfile result;
  result.points.reserve(path.size());
  result.s_values.reserve(path.size());
  double dist_sum = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      dist_sum +=
        autoware::universe_utils::calcDistance2d(path[i - 1].point.pose, path[i].point.pose);
    }
    double lim = path[i].point.longitudinal_velocity_mps;
    if (lim < g_min_velocity_threshold) lim = default_limit;

    // 1. Hard Stop
    if (stop_dist_limit >= 0.0 && dist_sum >= stop_dist_limit) {
      lim = 0.0;
    } else if (stop_dist_limit >= 0.0) {
      // 2. Analytical Limit (v^2 = 2*a*d)
      double dist_left = stop_dist_limit - dist_sum;
      // Calculate max allowed speed to stop comfortably from here
      double braking_speed = std::sqrt(2.0 * std::abs(min_decel) * dist_left);
      lim = std::min(lim, braking_speed);
    }

    result.points.push_back({dist_sum, 0.0, 0.0, lim});
    result.s_values.push_back(dist_sum);
  }
  result.total_length = dist_sum;
  return result;
}

std::vector<VelocityPoint> apply_jerk_filter(
  const std::vector<VelocityPoint> & input, double start_v, double start_a, double max_acc,
  double max_jerk)
{
  if (input.empty()) return {};
  std::vector<VelocityPoint> result;
  result.reserve(input.size());
  double curr_v = start_v;
  double curr_a = start_a;
  auto p0 = input[0];
  p0.v = curr_v;
  p0.a = curr_a;
  result.push_back(p0);
  for (size_t i = 1; i < input.size(); ++i) {
    const double ds = std::abs(input[i].s - input[i - 1].s);
    const double v_lim = input[i].v_limit;
    const double max_dt = std::pow(6.0 * ds / std::abs(max_jerk), 1.0 / 3.0);
    const double dt = std::min(ds / std::max(curr_v, g_min_velocity_threshold), max_dt);
    if (curr_a + max_jerk * dt >= max_acc) {
      const double tmp_jerk = std::min((max_acc - curr_a) / dt, max_jerk);
      curr_v = curr_v + curr_a * dt + 0.5 * tmp_jerk * dt * dt;
      curr_a = max_acc;
    } else {
      curr_v = curr_v + curr_a * dt + 0.5 * max_jerk * dt * dt;
      curr_a = curr_a + max_jerk * dt;
    }
    if (curr_v > v_lim) {
      curr_v = v_lim;
      curr_a = 0.0;
    }
    if (curr_v < 0.0) curr_v = 0.0;
    auto p = input[i];
    p.v = curr_v;
    p.a = curr_a;
    result.push_back(p);
  }
  return result;
}

std::vector<VelocityPoint> generate_backward_profile(
  const std::vector<VelocityPoint> & base_profile, double total_length, double target_stop_dist,
  double min_acc, double min_jerk)
{
  auto rev_input = base_profile;
  std::reverse(rev_input.begin(), rev_input.end());

  // Safety buffer to prevent overshoot
  constexpr double buffer = 0.5;
  double effective_start_s = total_length - std::max(0.0, target_stop_dist - buffer);

  for (auto & p : rev_input) {
    p.s = total_length - p.s;
  }

  std::vector<VelocityPoint> result;
  result.reserve(rev_input.size());

  size_t start_idx = 0;
  for (size_t i = 0; i < rev_input.size(); ++i) {
    if (rev_input[i].s >= effective_start_s) {
      start_idx = i;
      break;
    }
  }

  for (size_t i = 0; i < start_idx; ++i) {
    auto p = rev_input[i];
    p.v = 0.0;
    p.a = 0.0;
    result.push_back(p);
  }

  if (start_idx < rev_input.size()) {
    std::vector<VelocityPoint> active_segment;
    active_segment.reserve(rev_input.size() - start_idx);
    for (size_t i = start_idx; i < rev_input.size(); ++i) {
      active_segment.push_back(rev_input[i]);
    }
    auto filtered_segment =
      apply_jerk_filter(active_segment, 0.0, 0.0, std::abs(min_acc), std::abs(min_jerk));
    result.insert(result.end(), filtered_segment.begin(), filtered_segment.end());
  }

  std::reverse(result.begin(), result.end());
  for (auto & p : result) {
    p.s = total_length - p.s;
    p.a *= -1.0;
  }
  return result;
}

void bridge_emergency_profile(
  std::vector<VelocityPoint> & merged, const std::vector<VelocityPoint> & bwd_target, double v0,
  double a0, const Constraint & limit_constraints)
{
  double curr_v = v0;
  double curr_a = a0;
  size_t i = 0;
  while (i < merged.size() && curr_v > bwd_target[i].v) {
    merged[i].v = curr_v;
    merged[i].a = curr_a;
    if (i + 1 >= merged.size()) break;
    double ds = merged[i + 1].s - merged[i].s;
    double min_jerk = limit_constraints.min_jerk;
    double min_acc = limit_constraints.min_acc;
    double max_dt = std::pow(6.0 * ds / std::abs(min_jerk), 1.0 / 3.0);
    double dt = std::min(ds / std::max(curr_v, g_min_velocity_threshold), max_dt);
    if (curr_a + min_jerk * dt < min_acc) {
      double tmp_jerk = std::max((min_acc - curr_a) / dt, min_jerk);
      curr_v = curr_v + curr_a * dt + 0.5 * tmp_jerk * dt * dt;
      curr_a = std::max(curr_a + tmp_jerk * dt, min_acc);
    } else {
      curr_v = curr_v + curr_a * dt + 0.5 * min_jerk * dt * dt;
      curr_a = curr_a + min_jerk * dt;
    }
    i++;
  }
  for (; i < merged.size(); ++i) merged[i] = bwd_target[i];
}

std::vector<VelocityPoint> merge_profiles(
  const std::vector<VelocityPoint> & fwd, const std::vector<VelocityPoint> & bwd, double v0,
  double a0, const TrajectoryGenerationParams & params)
{
  std::vector<VelocityPoint> merged = fwd;
  if (v0 > bwd[0].v) {
    bridge_emergency_profile(merged, bwd, v0, a0, params.limit);
  } else {
    for (size_t i = 0; i < merged.size(); ++i) {
      merged[i] = (fwd[i].v < bwd[i].v) ? fwd[i] : bwd[i];
    }
  }
  return merged;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resample_to_time_domain(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & reference_path,
  const std::vector<double> & path_s, const std::vector<VelocityPoint> & velocity_profile,
  double v0, const TrajectoryGenerationParams & params)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory;
  trajectory.reserve(static_cast<size_t>(params.duration / params.time_step) + 5);

  double total_length = path_s.back();
  double t = 0.0;
  double s = 0.0;
  double v_curr = v0;

  double stop_horizon = total_length;
  if (params.temporary_stop_distance >= 0.0) {
    stop_horizon = std::min(stop_horizon, params.temporary_stop_distance);
  }

  while (s < total_length && t < g_safety_timeout) {
    auto traj_pose_opt = interp_geometry(reference_path, path_s, s);
    if (!traj_pose_opt) break;

    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = *traj_pose_opt;

    double delay_dist = std::max(0.0, v_curr * params.system_delay);
    double s_target = std::min(s + delay_dist, total_length);

    auto state_opt = interp_profile(velocity_profile, s_target);
    if (!state_opt) break;

    point.time_from_start = rclcpp::Duration::from_seconds(t);
    point.longitudinal_velocity_mps = static_cast<float>(state_opt->v);
    point.acceleration_mps2 = static_cast<float>(state_opt->a);
    point.front_wheel_angle_rad = 0.0f;

    trajectory.push_back(point);

    v_curr = state_opt->v;
    s += v_curr * params.time_step;
    t += params.time_step;

    if (s >= stop_horizon) {
      if (std::abs(s - stop_horizon) > 1e-3) {
        auto end_pose = interp_geometry(reference_path, path_s, stop_horizon);
        autoware_planning_msgs::msg::TrajectoryPoint end_point;
        end_point.pose = *end_pose;
        end_point.time_from_start = rclcpp::Duration::from_seconds(t);
        end_point.longitudinal_velocity_mps = 0.0f;
        end_point.acceleration_mps2 = 0.0f;
        trajectory.push_back(end_point);
      }
      break;
    }

    if (v_curr < g_min_velocity_threshold && t > params.duration * 2.0) break;
  }

  return trajectory;
}

}  // namespace

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> generate_trajectory(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & reference_path,
  const double current_velocity, const double current_acceleration,
  const TrajectoryGenerationParams & params)
{
  if (reference_path.empty()) return {};

  // Pass min_acc to create analytical limit
  const auto spatial = create_spatial_profile(
    reference_path, params.map_velocity_limit, params.temporary_stop_distance,
    params.normal.min_acc);

  const double v0 = std::max(current_velocity, g_min_start_speed);
  const auto fwd_profile = apply_jerk_filter(
    spatial.points, v0, current_acceleration, params.normal.max_acc, params.normal.max_jerk);

  double effective_stop =
    (params.temporary_stop_distance >= 0.0) ? params.temporary_stop_distance : spatial.total_length;
  effective_stop = std::min(effective_stop, spatial.total_length);

  const auto bwd_profile = generate_backward_profile(
    spatial.points, spatial.total_length, effective_stop, params.normal.min_acc,
    params.normal.min_jerk);

  const auto merged_profile =
    merge_profiles(fwd_profile, bwd_profile, v0, current_acceleration, params);

  return resample_to_time_domain(reference_path, spatial.s_values, merged_profile, v0, params);
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> generate_stop_and_go_sequence(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & reference_path,
  const double current_velocity, const double current_acceleration, double dist_to_stop,
  double stop_duration, const TrajectoryGenerationParams & params)
{
  if (reference_path.empty()) return {};

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> full_traj;

  // --- LEG 1 ---
  TrajectoryGenerationParams leg1_params = params;
  leg1_params.temporary_stop_distance = dist_to_stop;
  leg1_params.duration = 20.0;
  // Use NORMAL constraints for planned stops to prevent aggressive braking
  leg1_params.limit = leg1_params.normal;

  auto traj_1 =
    generate_trajectory(reference_path, current_velocity, current_acceleration, leg1_params);

  if (traj_1.empty()) return {};
  full_traj = traj_1;

  // --- LEG 2 (WAIT) ---
  auto stop_pose = full_traj.back();
  double t_accum = rclcpp::Duration(stop_pose.time_from_start).seconds();
  double t_resume = t_accum + stop_duration;

  for (double t = t_accum + params.time_step; t < t_resume; t += params.time_step) {
    auto p = stop_pose;
    p.time_from_start = rclcpp::Duration::from_seconds(t);
    p.longitudinal_velocity_mps = 0.0;
    p.acceleration_mps2 = 0.0;
    full_traj.push_back(p);
  }

  // --- LEG 3 (RESUME) ---
  double s = 0.0;
  size_t split_idx = 0;
  for (size_t i = 1; i < reference_path.size(); ++i) {
    s += autoware::universe_utils::calcDistance2d(
      reference_path[i - 1].point.pose, reference_path[i].point.pose);
    if (s >= dist_to_stop) {
      split_idx = i;
      break;
    }
  }

  if (split_idx < reference_path.size() - 2) {
    std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> sub_path;
    for (size_t i = split_idx; i < reference_path.size(); ++i) {
      sub_path.push_back(reference_path[i]);
    }

    TrajectoryGenerationParams leg2_params = params;
    leg2_params.temporary_stop_distance = -1.0;
    leg2_params.duration = 10.0;

    auto traj_2 = generate_trajectory(sub_path, 0.0, 0.0, leg2_params);

    for (auto & p : traj_2) {
      double t_local = rclcpp::Duration(p.time_from_start).seconds();
      if (t_local < 1e-3) continue;
      p.time_from_start = rclcpp::Duration::from_seconds(t_local + t_resume);
      full_traj.push_back(p);
    }
  }

  return full_traj;
}

builtin_interfaces::msg::Time second_to_stamp(double time_seconds)
{
  builtin_interfaces::msg::Time msg;

  // Extract integer seconds
  msg.sec = static_cast<int32_t>(time_seconds);

  // Extract remaining nanoseconds
  // (time - sec) gives the fractional part, * 1e9 converts to nanos
  msg.nanosec = static_cast<uint32_t>((time_seconds - msg.sec) * 1e9);

  return msg;
}

double calc_forward_length(
  const route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & curr_pose,
  const double default_forward_path_length)
{
  const auto ego_arc_length = lanelet::utils::getArcCoordinates(lanelet_sequence, curr_pose).length;
  double s_forward = ego_arc_length + default_forward_path_length;

  if (route_handler.isDeadEndLanelet(lanelet_sequence.back())) {
    const auto lane_length =
      lanelet::geometry::length2d(lanelet::LaneletSequence(lanelet_sequence));
    s_forward = std::clamp(s_forward, 0.0, lane_length);
  }

  if (route_handler.isInGoalRouteSection(lanelet_sequence.back())) {
    const auto goal_arc_coordinates =
      lanelet::utils::getArcCoordinates(lanelet_sequence, route_handler.getGoalPose());
    s_forward = std::clamp(s_forward, 0.0, goal_arc_coordinates.length);
  }

  return s_forward;
}

tl::expected<lanelet::ConstLanelets, std::string> get_lanelet_sequence(
  const route_handler::RouteHandler & route_handler, const geometry_msgs::msg::Pose & curr_pose,
  const double backward_sequence_length, const double forward_sequence_length)
{
  lanelet::ConstLanelet curr_lane;
  if (!route_handler.getClosestLaneletWithinRoute(curr_pose, &curr_lane)) {
    return tl::make_unexpected("failed to find closest lanelet within route");
    return {};
  }
  auto lanelet_sequence = route_handler.getLaneletSequence(
    curr_lane, curr_pose, backward_sequence_length, forward_sequence_length);

  if (lanelet_sequence.empty()) {
    return tl::make_unexpected("lanelet sequence is empty");
  }

  return lanelet_sequence;
}

double calc_segment_time(
  const autoware_planning_msgs::msg::PathPoint & curr,
  const autoware_planning_msgs::msg::PathPoint & prev)
{
  double dist = autoware::universe_utils::calcDistance2d(curr.pose.position, prev.pose.position);

  double avg_vel =
    (std::abs(curr.longitudinal_velocity_mps) + std::abs(prev.longitudinal_velocity_mps)) / 2.0;

  avg_vel = std::max(avg_vel, 0.01);

  return dist / avg_vel;
}

float calc_steering_angle(const autoware_planning_msgs::msg::PathPoint & p, double wheel_base)
{
  // Avoid noise when stopped
  if (std::abs(p.longitudinal_velocity_mps) < 0.01) {
    return 0.0f;
  }
  // curvature = omega / v
  double curvature = p.heading_rate_rps / p.longitudinal_velocity_mps;
  return static_cast<float>(std::atan(wheel_base * curvature));
}

// --- 3. Helper: Calculate Acceleration (Differentiation) ---
float calc_acc(
  const autoware_planning_msgs::msg::TrajectoryPoint & curr,
  const autoware_planning_msgs::msg::TrajectoryPoint & next)
{
  double v_next = next.longitudinal_velocity_mps;
  double v_curr = curr.longitudinal_velocity_mps;

  double t_next = rclcpp::Duration(next.time_from_start).seconds();
  double t_curr = rclcpp::Duration(curr.time_from_start).seconds();

  double dt = std::max(t_next - t_curr, 0.001);  // Prevent divide by zero

  return static_cast<float>((v_next - v_curr) / dt);
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> convert_to_trajectory(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path_points,
  double wheel_base)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj_points;
  if (path_points.empty()) return traj_points;

  traj_points.reserve(path_points.size());
  double accumulated_time = 0.0;

  // --- Pass 1: Populate Pose, Velocity, Time, and Steering ---
  for (size_t i = 0; i < path_points.size(); ++i) {
    const auto & src = path_points[i].point;
    autoware_planning_msgs::msg::TrajectoryPoint dst;

    // Copy direct fields
    dst.pose = src.pose;
    dst.longitudinal_velocity_mps = src.longitudinal_velocity_mps;
    dst.lateral_velocity_mps = src.lateral_velocity_mps;
    dst.heading_rate_rps = src.heading_rate_rps;
    dst.rear_wheel_angle_rad = 0.0;  // Default

    // Calculate Time
    if (i > 0) {
      accumulated_time += calc_segment_time(src, path_points[i - 1].point);
    }
    dst.time_from_start = rclcpp::Duration::from_seconds(accumulated_time);
    dst.front_wheel_angle_rad = calc_steering_angle(src, wheel_base);
    dst.acceleration_mps2 = 0.0;

    traj_points.push_back(dst);
  }

  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (i < traj_points.size() - 1) {
      traj_points[i].acceleration_mps2 = calc_acc(traj_points[i], traj_points[i + 1]);
    } else if (i > 0) {
      traj_points[i].acceleration_mps2 = traj_points[i - 1].acceleration_mps2;
    }
  }

  return traj_points;
}

// Helper to calculate time-based velocity limit for Stop & Go mode
/**
 * @brief Calculates the time-based velocity limit for Stop & Go behavior.
 * Why: To enforce stopping at specific time intervals regardless of position.
 */
double get_stop_and_go_limit(double t, const TrajectoryGenerationParams & params)
{
  if (params.mode != TrajectoryMode::STOP_AND_GO) {
    return std::numeric_limits<double>::max();
  }

  // Phase 1: Move (0 -> move_duration)
  // Why: We must decelerate to 0 by the time t reaches move_duration.
  // We use a linear ramp down: v_limit = max_decel * time_left
  if (t < params.move_duration) {
    double time_left = params.move_duration - t;
    double max_decel = std::abs(params.normal.min_acc);
    return max_decel * time_left;
  }

  // Phase 2: Stop (move_duration -> move + stop)
  if (t < params.move_duration + params.stop_duration) {
    return 0.0;
  }

  // Phase 3: Resume (after move + stop)
  return std::numeric_limits<double>::max();
}

bool is_near_goal(
  const geometry_msgs::msg::Pose & curr_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  return autoware::universe_utils::calcDistance2d(curr_pose.position, goal_pose.position) < 2.0;
}
}  // namespace autoware::manual_trajectory::helper
