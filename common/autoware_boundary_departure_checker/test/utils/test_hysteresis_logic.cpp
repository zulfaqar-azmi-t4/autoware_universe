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

#include <gtest/gtest.h>

#include <optional>

namespace autoware::boundary_departure_checker
{
namespace
{
using EvaluationResult = std::optional<Side<std::optional<CriticalPointPair>>>;

UncrossableBoundaryDepartureParam make_param()
{
  UncrossableBoundaryDepartureParam param;
  param.on_time_buffer_s = 0.15;
  param.off_time_buffer_s = 0.15;
  param.lateral_margin_m = 0.01;
  param.release_lateral_margin_m = 0.30;
  return param;
}

EvaluationResult make_left_departure(const DepartureType type)
{
  CriticalPointPair pair;
  pair.physical_departure_point.departure_type = type;
  pair.safety_buffer_start.departure_type = type;
  pair.physical_departure_point.time_from_start = 5.0;

  Side<std::optional<CriticalPointPair>> sides;
  sides.left = pair;
  return sides;
}

EvaluationResult make_no_departure()
{
  return Side<std::optional<CriticalPointPair>>{};
}
}  // namespace

TEST(HysteresisLogicTest, TestIsDepartureFree)
{
  Side<std::optional<CriticalPointPair>> sides;
  EXPECT_TRUE(severity_evaluator::is_departure_free(sides));

  CriticalPointPair approaching;
  approaching.physical_departure_point.departure_type = DepartureType::APPROACHING;
  sides.left = approaching;
  EXPECT_FALSE(severity_evaluator::is_departure_free(sides));
}

TEST(HysteresisLogicTest, TestEffectiveLateralMarginWidensWhileHeld)
{
  // Arrange:
  const auto param = make_param();
  HysteresisState state;

  // Act & Assert: with nothing held, the trigger margin applies.
  EXPECT_DOUBLE_EQ(calc_effective_lateral_margin(state, param), param.lateral_margin_m);

  // Act & Assert: once a critical verdict is held, the wider release margin applies.
  state =
    update_and_judge(state, make_left_departure(DepartureType::CRITICAL), param, 1.0).updated_state;
  EXPECT_DOUBLE_EQ(calc_effective_lateral_margin(state, param), param.release_lateral_margin_m);
}

TEST(HysteresisLogicTest, TestEffectiveLateralMarginNeverNarrows)
{
  // Arrange: a release margin smaller than the trigger margin is a misconfiguration.
  auto param = make_param();
  param.lateral_margin_m = 0.5;
  param.release_lateral_margin_m = 0.1;

  HysteresisState state =
    update_and_judge(HysteresisState{}, make_left_departure(DepartureType::CRITICAL), param, 1.0)
      .updated_state;

  // Act & Assert: the held margin is never narrower than the trigger margin.
  EXPECT_DOUBLE_EQ(calc_effective_lateral_margin(state, param), param.lateral_margin_m);
}

TEST(HysteresisLogicTest, TestCriticalHeldWhileApproachingRemains)
{
  // Arrange: a critical departure is raised while ego is moving.
  const auto param = make_param();
  HysteresisState state;

  const auto raised =
    update_and_judge(state, make_left_departure(DepartureType::CRITICAL), param, 1.0);
  ASSERT_EQ(raised.status, DepartureType::CRITICAL);
  state = raised.updated_state;

  // Act: ego brakes to a stop, so the same geometry now evaluates as APPROACHING because the
  // braking distance collapsed. Well past the off time buffer.
  const auto after_stop =
    update_and_judge(state, make_left_departure(DepartureType::APPROACHING), param, 10.0);

  // Assert: the verdict is held, so ego cannot resume into the same boundary.
  EXPECT_EQ(after_stop.status, DepartureType::CRITICAL);
  EXPECT_FALSE(after_stop.updated_state.critical_departure_history.all_empty());
}

TEST(HysteresisLogicTest, TestLatchDisabledRestoresSeverityOnlyRelease)
{
  // Arrange: the latch is switched off.
  auto param = make_param();
  param.latch_critical_until_clear = false;

  HysteresisState state =
    update_and_judge(HysteresisState{}, make_left_departure(DepartureType::CRITICAL), param, 1.0)
      .updated_state;

  // Act & Assert: the trigger margin still applies while a verdict is stored.
  EXPECT_DOUBLE_EQ(calc_effective_lateral_margin(state, param), param.lateral_margin_m);

  // Act & Assert: a drop to APPROACHING releases once the off time buffer elapses.
  const auto after_stop =
    update_and_judge(state, make_left_departure(DepartureType::APPROACHING), param, 10.0);
  EXPECT_EQ(after_stop.status, DepartureType::NONE);
  EXPECT_TRUE(after_stop.updated_state.critical_departure_history.all_empty());
}

TEST(HysteresisLogicTest, TestCriticalReleasedOnlyWhenDepartureFree)
{
  // Arrange:
  const auto param = make_param();
  HysteresisState state =
    update_and_judge(HysteresisState{}, make_left_departure(DepartureType::CRITICAL), param, 1.0)
      .updated_state;

  // Act & Assert: a departure free result within the off time buffer still holds.
  const auto within_buffer = update_and_judge(state, make_no_departure(), param, 1.1);
  EXPECT_EQ(within_buffer.status, DepartureType::CRITICAL);

  // Act & Assert: after the off time buffer the history clears and the status drops.
  const auto after_buffer = update_and_judge(state, make_no_departure(), param, 1.5);
  EXPECT_EQ(after_buffer.status, DepartureType::NONE);
  EXPECT_TRUE(after_buffer.updated_state.critical_departure_history.all_empty());
}

TEST(HysteresisLogicTest, TestApproachingAloneNeverRaisesCritical)
{
  // Arrange:
  const auto param = make_param();

  // Act:
  const auto result = update_and_judge(
    HysteresisState{}, make_left_departure(DepartureType::APPROACHING), param, 1.0);

  // Assert:
  EXPECT_EQ(result.status, DepartureType::NONE);
  EXPECT_TRUE(result.updated_state.critical_departure_history.all_empty());
}

TEST(HysteresisLogicTest, TestEmptyEvaluationClearsHistory)
{
  // Arrange:
  const auto param = make_param();
  HysteresisState state =
    update_and_judge(HysteresisState{}, make_left_departure(DepartureType::CRITICAL), param, 1.0)
      .updated_state;

  // Act: no evaluation result means no boundary was found near the trajectory.
  const auto result = update_and_judge(state, std::nullopt, param, 1.1);

  // Assert:
  EXPECT_EQ(result.status, DepartureType::NONE);
  EXPECT_TRUE(result.updated_state.critical_departure_history.all_empty());
}
}  // namespace autoware::boundary_departure_checker
