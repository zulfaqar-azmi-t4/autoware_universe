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

#include "autoware/boundary_departure_checker/detail/hysteresis_logic.hpp"

#include "autoware/boundary_departure_checker/detail/severity_evaluator.hpp"

#include <algorithm>

namespace autoware::boundary_departure_checker
{
double calc_effective_lateral_margin(
  const HysteresisState & state, const UncrossableBoundaryDepartureParam & param)
{
  // While a critical verdict is held, widen the margin so that a path which only just grazes the
  // boundary cannot release it. This makes the hysteresis asymmetric in distance, not only in time.
  if (!param.latch_critical_until_clear || state.critical_departure_history.all_empty()) {
    return param.lateral_margin_m;
  }
  return std::max(param.lateral_margin_m, param.release_lateral_margin_m);
}

HysteresisResult update_and_judge(
  const HysteresisState & state,
  const std::optional<Side<std::optional<CriticalPointPair>>> & evaluation_result,
  const UncrossableBoundaryDepartureParam & param, const double current_time_s)
{
  HysteresisResult result;
  result.updated_state = state;
  result.status = DepartureType::NONE;

  if (!evaluation_result.has_value()) {
    result.updated_state.last_no_critical_dpt_time = current_time_s;
    result.updated_state.critical_departure_history.for_each_side(
      [](auto & side) { side.clear(); });
    return result;
  }

  const bool current_is_critical = severity_evaluator::is_critical(*evaluation_result);

  if (current_is_critical) {
    const bool is_imminent_critical =
      evaluation_result->any_of_side([&param](const auto & side_value) {
        return side_value.has_value() && side_value->physical_departure_point.is_critical() &&
               side_value->physical_departure_point.time_from_start < param.on_time_buffer_s;
      });

    if (
      is_imminent_critical ||
      current_time_s - state.last_no_critical_dpt_time >= param.on_time_buffer_s) {
      result.updated_state.last_found_critical_dpt_time = current_time_s;
      result.updated_state.critical_departure_history.for_each_side(
        [](auto & side) { side.clear(); });
      evaluation_result->for_each([&](auto key_constant, auto & side_value) {
        if (side_value.has_value() && side_value->safety_buffer_start.is_critical()) {
          result.updated_state.critical_departure_history[key_constant.value].push_back(
            side_value->physical_departure_point);
        }
      });
      result.status = DepartureType::CRITICAL;
      return result;
    }
    return result;
  }

  if (!state.critical_departure_history.all_empty()) {
    // Hold the critical verdict until the boundary proximity itself clears. A drop to APPROACHING
    // only means ego slowed down, which is the effect of the verdict, not a recovery from it.
    if (
      param.latch_critical_until_clear &&
      !severity_evaluator::is_departure_free(*evaluation_result)) {
      result.status = DepartureType::CRITICAL;
      return result;
    }

    if (current_time_s - state.last_found_critical_dpt_time < param.off_time_buffer_s) {
      result.status = DepartureType::CRITICAL;
      return result;
    }

    result.updated_state.critical_departure_history.for_each_side(
      [](auto & side) { side.clear(); });
  }

  result.updated_state.last_no_critical_dpt_time = current_time_s;

  return result;
}

}  // namespace autoware::boundary_departure_checker
