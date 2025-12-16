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

#include "manual_trajectory.hpp"

#include "manual_trajectory_helper.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace autoware::manual_trajectory
{
ManualTrajectory::ManualTrajectory(const rclcpp::NodeOptions & node_options)
: Node("manual_trajectory", node_options)
{
  rclcpp::QoS qos{1};
  qos.transient_local();
  pub_traj_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/trajectory", qos);

  vehicle_info_ = vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  traj_gen_param_.time_step = 0.1;
  traj_gen_param_.duration = 10.0;

  const auto get_common_param = [this](std::string && ns) {
    Constraint constraints;
    constraints.max_acc =
      autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, ns + ".max_acc");
    constraints.min_acc =
      autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, ns + ".min_acc");
    constraints.max_jerk =
      autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, ns + ".max_jerk");
    constraints.min_jerk =
      autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, ns + ".min_jerk");
    return constraints;
  };

  traj_gen_param_.normal = get_common_param("normal");
  traj_gen_param_.limit = get_common_param("limit");

  auto period_s = std::chrono::duration<double>(1.0 / 50.0);
  timer_ = rclcpp::create_timer(this, this->get_clock(), period_s, [this]() { on_timer(); });
}

void ManualTrajectory::take_data()
{
  if (const auto msg = vector_map_subscriber_.take_data()) {
    route_handler_.setMap(*msg);
  }

  if (const auto msg = route_subscriber_.take_data()) {
    route_handler_.setRoute(*msg);
  }

  if (const auto msg = odometry_subscriber_.take_data()) {
    odometry_ptr_ = msg;
  }

  if (const auto msg = acceleration_subscriber_.take_data()) {
    acceleration_ptr_ = msg;
  }

  if (const auto msg = op_mode_state_subscriber_.take_data()) {
    op_mode_state_ptr_ = msg;
  }
}

std::optional<std::string> ManualTrajectory::has_invalid_data() const
{
  if (!route_handler_.isMapMsgReady()) {
    return {"vector map is not ready"};
  }

  if (!route_handler_.isHandlerReady()) {
    return {"route handler is not ready"};
  }

  if (!odometry_ptr_) {
    return {"odometry is not ready"};
  }

  if (!acceleration_ptr_) {
    return {"acceleration is not ready"};
  }

  return std::nullopt;
}

void ManualTrajectory::on_timer()
{
  take_data();
  if (const auto err = has_invalid_data()) {
    warn_throttle("%s", err->c_str());
    return;
  }

  auto traj_points_opt = generate_centerline_path();
  if (!traj_points_opt) {
    warn_throttle("failed to generate centerline path: %s", traj_points_opt.error().c_str());
    return;
  }

  autoware_planning_msgs::msg::Trajectory traj_msg;
  traj_msg.header.stamp = this->get_clock()->now();
  traj_msg.header.frame_id = "map";
  traj_msg.points = std::move(*traj_points_opt);
  pub_traj_->publish(traj_msg);
}

void ManualTrajectory::update_test_state(
  double current_vel, double ego_arc_len, const rclcpp::Time & now)
{
  // Initial Transition (Gated by Autonomous Mode)
  if (state_ == TestState::IDLE) {
    state_start_time_ = now;
    if (is_autonomous()) {
      state_ = TestState::DRIVING_1;

      // Define stop point relative to start
      double dist_to_stop = 30.0;
      cached_stop_arc_length_ = ego_arc_len + dist_to_stop;
      info("Test Start: IDLE -> DRIVING_1 (Target Stop: %.1fm)", cached_stop_arc_length_);
    }
  }

  // Safety Reset
  if (state_ != TestState::IDLE && !is_autonomous()) {
    state_ = TestState::IDLE;
    cached_stop_arc_length_ = -1.0;
    info("Autonomous disengaged. Reset -> IDLE");
  }

  // State Transitions
  switch (state_) {
    case TestState::DRIVING_1:
      // We transition based on TIME here for the state machine logic
      if ((now - state_start_time_).seconds() > 5.0) {
        state_ = TestState::BRAKING;
        state_start_time_ = now;
        info("Timer(5s): DRIVING_1 -> BRAKING");
      }
      break;

    case TestState::BRAKING:
      if (std::abs(current_vel) < 0.1) {
        state_ = TestState::STOPPED;
        state_start_time_ = now;
        info("Vehicle Stopped: BRAKING -> STOPPED");
      }
      break;

    case TestState::STOPPED:
      if (std::abs(current_vel) >= 0.1) {
        state_start_time_ = now;
      }
      if ((now - state_start_time_).seconds() > 5.0) {
        state_ = TestState::DRIVING_2;
        info("Wait(5s): STOPPED -> DRIVING_2");
      }
      break;

    default:
      break;
  }
}

tl::expected<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>, std::string>
ManualTrajectory::generate_centerline_path()
{
  const auto & curr_pose = odometry_ptr_->pose.pose;
  const double current_vel = odometry_ptr_->twist.twist.linear.x;
  const double current_acc = acceleration_ptr_->accel.accel.linear.x;
  const auto now = this->get_clock()->now();

  // 1. Generate Geometric Path
  constexpr double forward_sequence_length = 50.0;
  constexpr double backward_sequence_length = 0.0;

  auto lanelet_sequence_opt = helper::get_lanelet_sequence(
    route_handler_, curr_pose, backward_sequence_length, forward_sequence_length);
  if (!lanelet_sequence_opt) {
    return tl::make_unexpected(lanelet_sequence_opt.error());
  }
  const auto lanelet_sequence = lanelet_sequence_opt.value();

  const auto forward_path_length = helper::calc_forward_length(
    route_handler_, lanelet_sequence, curr_pose, forward_sequence_length);

  const auto ego_arc_coordinates = lanelet::utils::getArcCoordinates(lanelet_sequence, curr_pose);
  const double ego_arc_len = ego_arc_coordinates.length;

  const auto raw_path =
    route_handler_.getCenterLinePath(lanelet_sequence, ego_arc_len, forward_path_length, true);

  const auto resampled_path = autoware::motion_utils::resamplePath(raw_path, 0.1, true);

  // 2. State Machine Logic
  update_test_state(current_vel, ego_arc_len, now);

  // 3. Map Global Stop Decision to Local Path Limit
  double local_stop_dist = -1.0;

  // === FIX: Calculate limits in DRIVING_1 too ===
  // This allows the planner to see the stop point immediately.
  bool stop_is_active =
    (state_ == TestState::DRIVING_1 || state_ == TestState::BRAKING ||
     state_ == TestState::STOPPED);

  if (stop_is_active && cached_stop_arc_length_ > 0.0) {
    size_t ego_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      resampled_path.points, curr_pose, 3.0, 1.0);

    double ego_s_in_vector =
      autoware::motion_utils::calcSignedArcLength(resampled_path.points, 0, ego_idx);

    double dist_remaining = cached_stop_arc_length_ - ego_arc_len;
    local_stop_dist = std::max(0.0, ego_s_in_vector + dist_remaining);
  }

  // 4. Generate Trajectory
  // === FIX: Use Sequence Generator if a stop is active ===
  if (stop_is_active && local_stop_dist >= 0.0) {
    // This function stitches Leg 1 (Stop) + Wait + Leg 2 (Resume)
    return helper::generate_stop_and_go_sequence(
      resampled_path.points, current_vel, current_acc, local_stop_dist, 5.0 /*stop_duration*/,
      traj_gen_param_);
  }

  // Fallback (IDLE / DRIVING_2) - Normal Driving
  traj_gen_param_.temporary_stop_distance = -1.0;
  return helper::generate_trajectory(
    resampled_path.points, current_vel, current_acc, traj_gen_param_);
}

bool ManualTrajectory::is_autonomous() const
{
  if (!op_mode_state_ptr_) return false;

  using autoware_adapi_v1_msgs::msg::OperationModeState;
  return (op_mode_state_ptr_->mode == OperationModeState::AUTONOMOUS) &&
         op_mode_state_ptr_->is_autoware_control_enabled;
}
}  // namespace autoware::manual_trajectory

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::manual_trajectory::ManualTrajectory)
