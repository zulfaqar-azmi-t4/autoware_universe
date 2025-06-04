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

#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

namespace autoware::motion_velocity_planner
{
struct BehaviorTriggerThreshold
{
  double decel_mp2{-1.0};
  double brake_delay_s{1.0};
  double dist_error_m{0.25};
};

struct SlowDownBehavior
{
  double velocity_mps{0.0};
};

struct BoundaryBehaviorTrigger
{
  bool enable{true};
  BoundaryThreshold th_dist_to_boundary_m{
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  BehaviorTriggerThreshold th_trigger;
};

struct SlowDownNearBoundaryTrigger : BoundaryBehaviorTrigger
{
  SlowDownBehavior slow_down_behavior;

  SlowDownNearBoundaryTrigger() = default;
  explicit SlowDownNearBoundaryTrigger(BoundaryBehaviorTrigger bound_behavior)
  : BoundaryBehaviorTrigger(bound_behavior)
  {
  }
};

struct PredictedPathFootprint
{
  double scale{1.0};
  double extra_margin_m{0.0};
  double resample_interval_m{0.3};
};

struct Output
{
  std::unordered_map<std::string, double> processing_time_map;
  AbnormalityType<Footprints> footprints;
  AbnormalityType<EgoSides> ego_sides_from_fps;
  BoundarySideWithIdx boundary_segments;
  AbnormalityType<SideToBoundPojections> side_to_bound_projections;
  AbnormalityType<DepartureStatuses> departure_statuses;
  std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> slow_down_interval;

  trajectory::Trajectory<TrajectoryPoint> aw_ref_traj;
  trajectory::Trajectory<TrajectoryPoint> aw_ego_traj;

  DepartureIntervals departure_intervals;
  DeparturePoints departure_points;
  CriticalDeparturePoints critical_departure_points;
  bool is_critical_departing{false};
};

struct NodeParam
{
  double th_data_timeout_s{1.0};
  BDCParam bdc_param;

  double th_departure_point_lifetime_s{1.0};
  double th_dist_hysteresis_m{2.0};

  SlowDownNearBoundaryTrigger slow_down_near_boundary;
  BoundaryBehaviorTrigger slow_down_before_departure;
  BoundaryBehaviorTrigger stop_before_departure;

  PredictedPathFootprint pred_path_footprint;

  NodeParam() = default;
  explicit NodeParam(rclcpp::Node & node)
  {
    const std::string module_name{"boundary_departure_prevention."};
    th_data_timeout_s = get_or_declare_parameter<double>(node, module_name + "th_data_timeout_s");
    bdc_param.boundary_types_to_detect = get_or_declare_parameter<std::vector<std::string>>(
      node, module_name + "boundary_types_to_detect");
    bdc_param.th_max_lateral_query_num =
      get_or_declare_parameter<int>(node, module_name + "th_max_lateral_query_num");
    bdc_param.abnormality_types_to_compensate = std::invoke([&node, &module_name]() {
      const std::string abnormality_ns{module_name + "abnormality_types_to_compensate."};
      const auto compensate_normal =
        get_or_declare_parameter<bool>(node, abnormality_ns + "normal");
      const auto compensate_steering =
        get_or_declare_parameter<bool>(node, abnormality_ns + "steering");
      const auto compensate_localization =
        get_or_declare_parameter<bool>(node, abnormality_ns + "localization");
      const auto compensate_longitudinal =
        get_or_declare_parameter<bool>(node, abnormality_ns + "longitudinal");

      std::vector<AbnormalityKeys> abnormality_types_to_compensate;
      abnormality_types_to_compensate.reserve(4);
      if (compensate_normal) {
        abnormality_types_to_compensate.emplace_back(AbnormalityKeys::NORMAL);
      }

      if (compensate_steering) {
        abnormality_types_to_compensate.emplace_back(AbnormalityKeys::STEERING);
      }

      if (compensate_localization) {
        abnormality_types_to_compensate.emplace_back(AbnormalityKeys::LOCALIZATION);
      }

      if (compensate_longitudinal) {
        abnormality_types_to_compensate.emplace_back(AbnormalityKeys::LONGITUDINAL);
      }

      return abnormality_types_to_compensate;
    });

    auto boundary_behaviour_trigger_param = [&node,
                                             &module_name](const std::string & trigger_type_str) {
      BoundaryThreshold th_dist_to_boundary_m;
      const std::string ns{module_name + trigger_type_str + "."};
      const std::string ns_bound = ns + "th_dist_to_boundary_m.";
      th_dist_to_boundary_m.left = get_or_declare_parameter<double>(node, ns_bound + "left");
      th_dist_to_boundary_m.right = get_or_declare_parameter<double>(node, ns_bound + "right");

      BehaviorTriggerThreshold th_behavior;
      const std::string ns_behavior{ns + "th_trigger."};
      th_behavior.decel_mp2 = get_or_declare_parameter<double>(node, ns_behavior + "decel_mp2");
      th_behavior.brake_delay_s =
        get_or_declare_parameter<double>(node, ns_behavior + "brake_delay_s");
      th_behavior.dist_error_m =
        get_or_declare_parameter<double>(node, ns_behavior + "dist_error_m");

      BoundaryBehaviorTrigger trigger;
      trigger.enable = get_or_declare_parameter<bool>(node, ns + "enable");
      trigger.th_dist_to_boundary_m = th_dist_to_boundary_m;
      trigger.th_trigger = th_behavior;

      return trigger;
    };

    auto slow_down_behaviour_trigger_param =
      [&node, &module_name,
       &boundary_behaviour_trigger_param](const std::string & trigger_type_str) {
        SlowDownNearBoundaryTrigger trigger(boundary_behaviour_trigger_param(trigger_type_str));
        SlowDownBehavior slow_behavior;
        const std::string ns{module_name + trigger_type_str + "."};
        const std::string ns_slow = ns + "slow_down_behavior.";
        slow_behavior.velocity_mps =
          get_or_declare_parameter<double>(node, ns_slow + "velocity_kmh") / 3.6;
        return trigger;
      };
    slow_down_near_boundary = slow_down_behaviour_trigger_param("slow_down_near_boundary");
    slow_down_before_departure = boundary_behaviour_trigger_param("slow_down_before_departure");
    stop_before_departure = boundary_behaviour_trigger_param("stop_before_departure");

    pred_path_footprint = std::invoke([&node, &module_name]() {
      PredictedPathFootprint param;
      const std::string ns{module_name + "predicted_path_footprint."};
      param.scale = get_or_declare_parameter<double>(node, ns + "scale");
      param.extra_margin_m = get_or_declare_parameter<double>(node, ns + "extra_margin_m");
      param.resample_interval_m =
        get_or_declare_parameter<double>(node, ns + "resample_interval_m");
      return param;
    });

    const std::string ns_fp_envelop{module_name + "footprint_envelop."};
    bdc_param.footprint_envelop.lon_m =
      get_or_declare_parameter<double>(node, ns_fp_envelop + "lon_m");
    bdc_param.footprint_envelop.lat_m =
      get_or_declare_parameter<double>(node, ns_fp_envelop + "lat_m");

    const std::string ns_lon_tracking{module_name + "lon_tracking."};
    bdc_param.lon_tracking.scale =
      get_or_declare_parameter<double>(node, ns_lon_tracking + "scale");
    bdc_param.lon_tracking.extra_margin_m =
      get_or_declare_parameter<double>(node, ns_lon_tracking + "extra_margin_m");
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // PARAMETERS_HPP_
