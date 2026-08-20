// Copyright 2026 TIER IV, Inc. All rights reserved.
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

#include "../src/parameters.hpp"
#include "../src/types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
// declared here because collisions.cpp is compiled into the module library without a matching
// header
Collision calculate_collision(
  const TimeOverlapInterval & ego, const TimeOverlapInterval & object, const Parameters & params);
void calculate_overlapping_collision(
  Collision & c, const TimeOverlapInterval & ego, const TimeOverlapInterval & object,
  const Parameters & params);
struct TimeOverlapIntervalPair
{
  TimeOverlapInterval ego;
  TimeOverlapInterval object;
};
std::vector<TimeOverlapIntervalPair> calculate_overlap_intervals(
  std::vector<FootprintIntersection> intersections);

namespace
{
Parameters make_params()
{
  Parameters params;
  params.collision_time_margin = 1.0;
  params.collision_same_direction_angle_threshold = 0.785;      // [rad] 45 degrees
  params.collision_opposite_direction_angle_threshold = 0.785;  // [rad] 45 degrees
  params.ignore_collision_conditions.if_ego_arrives_first.enable = false;
  params.ignore_collision_conditions.if_ego_arrives_first.margin.ego_enter_times = {0.0, 10.0};
  params.ignore_collision_conditions.if_ego_arrives_first.margin.time_margins = {0.0, 0.0};
  params.ignore_collision_conditions.if_ego_arrives_first.max_overlap_duration = 0.0;
  params.ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.enable = false;
  params.ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop
    .calculated_stop_time_limit = 0.0;
  return params;
}

FootprintIntersection make_intersection(
  const double ego_time, const double object_time, const double arc_length)
{
  FootprintIntersection fi;
  fi.ego_time = ego_time;
  fi.object_time = object_time;
  fi.arc_length = arc_length;
  fi.yaw_diff = M_PI;  // ego and the object move in opposite directions
  fi.ego_vel = 5.0;
  fi.vel_diff = 10.0;
  return fi;
}
}  // namespace

// values taken from the incident bag at 2026-08-18 10:18:01.749 JST, where the ego overlap interval
// collapsed to the single time 11.1s and the reported collision time was -nan
TEST(TestCollisions, degenerate_ego_overlap_gives_finite_collision_time)
{
  const auto params = make_params();
  const auto ego_intersection = make_intersection(11.1, 9.96, 30.0);
  const TimeOverlapInterval ego(11.1, 11.1, ego_intersection, ego_intersection);
  const TimeOverlapInterval object(
    9.96, 12.0, make_intersection(11.1, 9.96, 30.0), make_intersection(11.1, 12.0, 30.0));

  const auto c = calculate_collision(ego, object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_TRUE(std::isfinite(c.ego_collision_time)) << "collision time = " << c.ego_collision_time;
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.1);
}

TEST(TestCollisions, zero_length_ego_overlap_gives_finite_collision_time)
{
  const auto params = make_params();
  // the two ego intersections are distinct in time but at the same arc length
  const TimeOverlapInterval ego(
    11.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 11.0, 30.0));
  const TimeOverlapInterval object(
    10.0, 11.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 11.0, 30.0));

  const auto c = calculate_collision(ego, object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_TRUE(std::isfinite(c.ego_collision_time)) << "collision time = " << c.ego_collision_time;
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.0);
}

TEST(TestCollisions, zero_object_overlap_duration_gives_finite_collision_time)
{
  const auto params = make_params();
  const TimeOverlapInterval ego(
    11.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 10.0, 40.0));
  const auto object_intersection = make_intersection(11.0, 10.0, 30.0);
  const TimeOverlapInterval object(10.0, 10.0, object_intersection, object_intersection);

  const auto c = calculate_collision(ego, object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_TRUE(std::isfinite(c.ego_collision_time)) << "collision time = " << c.ego_collision_time;
}

TEST(TestCollisions, regular_opposite_direction_overlap_is_unchanged)
{
  const auto params = make_params();
  // ego covers 10m of overlap in 1s, the object covers it in 2s
  const TimeOverlapInterval ego(
    11.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 12.0, 40.0));
  const TimeOverlapInterval object(
    10.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 12.0, 40.0));

  const auto c = calculate_collision(ego, object, params);

  EXPECT_EQ(c.type, collision);
  // lon_buffer = min(10, 4) = 4, ego_vel = 10, obj_vel = 5 -> (10-4)/15 = 0.4
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.4);
}

// the object's predicted path ends inside the ego footprint, so its last intersection carries the
// position inside_front_polygon and toggles neither the front nor the rear overlap flag
TEST(TestCollisions, path_ending_inside_the_footprint_creates_a_degenerate_overlap)
{
  std::vector<FootprintIntersection> intersections;
  // the object enters and exits the rear part of the footprint, closing a first overlap
  auto entering = make_intersection(9.0, 0.5, 20.0);
  entering.position = rear_left;
  intersections.push_back(entering);
  auto exiting = make_intersection(10.0, 5.0, 25.0);
  exiting.position = rear_right;
  intersections.push_back(exiting);
  // the path then ends inside the front polygon, at the 12s horizon of the predicted path
  auto path_end = make_intersection(11.1, 12.0, 33.0);
  path_end.position = inside_front_polygon;
  intersections.push_back(path_end);

  const auto intervals = calculate_overlap_intervals(intersections);

  ASSERT_EQ(intervals.size(), 2UL);
  EXPECT_FALSE(intervals[0].ego.is_open);
  const auto & degenerate = intervals[1];
  EXPECT_TRUE(degenerate.ego.is_open);
  EXPECT_TRUE(degenerate.object.is_open);
  EXPECT_DOUBLE_EQ(degenerate.ego.from, degenerate.ego.to);
  EXPECT_DOUBLE_EQ(
    degenerate.ego.first_intersection.arc_length, degenerate.ego.last_intersection.arc_length);
  EXPECT_DOUBLE_EQ(
    degenerate.object.first_intersection.object_time,
    degenerate.object.last_intersection.object_time);
}

// route (b): no inside_* tag at all. The last intersection opens an overlap that the loop never
// closes, so the trailing create_overlap at collisions.cpp:295 is called with entering_id already
// sitting on the last element.
TEST(TestCollisions, unclosed_trailing_overlap_is_also_degenerate)
{
  std::vector<FootprintIntersection> intersections;
  auto entering = make_intersection(9.0, 0.5, 20.0);
  entering.position = rear_left;
  intersections.push_back(entering);
  auto exiting = make_intersection(10.0, 5.0, 25.0);
  exiting.position = rear_right;
  intersections.push_back(exiting);
  // opens a new overlap and the list ends
  auto unclosed = make_intersection(10.9, 12.0, 31.0);
  unclosed.position = front_left;
  intersections.push_back(unclosed);

  const auto intervals = calculate_overlap_intervals(intersections);

  ASSERT_EQ(intervals.size(), 2UL);
  EXPECT_FALSE(intervals[0].ego.is_open);
  const auto & degenerate = intervals[1];
  EXPECT_TRUE(degenerate.ego.is_open);
  EXPECT_TRUE(degenerate.object.is_open);
  EXPECT_DOUBLE_EQ(degenerate.ego.from, degenerate.ego.to);
  EXPECT_DOUBLE_EQ(
    degenerate.ego.first_intersection.arc_length, degenerate.ego.last_intersection.arc_length);
  EXPECT_DOUBLE_EQ(
    degenerate.object.first_intersection.object_time,
    degenerate.object.last_intersection.object_time);
}

// an interval carries its open state through the merge in combine_time_overlap_intervals
TEST(TestCollisions, expand_propagates_the_open_state)
{
  const auto fi = make_intersection(11.0, 10.0, 30.0);
  TimeOverlapInterval closed(11.0, 12.0, fi, make_intersection(12.0, 12.0, 40.0));
  const TimeOverlapInterval open(11.5, 13.0, fi, fi, true);

  ASSERT_FALSE(closed.is_open);
  closed.expand(open);

  EXPECT_TRUE(closed.is_open);
  EXPECT_DOUBLE_EQ(closed.to, 13.0);
}

// an open overlap wide enough to divide safely still gets the entry time, because the span is not
// something that was observed
TEST(TestCollisions, open_overlap_with_a_wide_span_still_uses_the_entry_time)
{
  const auto params = make_params();
  const TimeOverlapInterval ego(
    11.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 12.0, 40.0), true);
  const TimeOverlapInterval object(
    10.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 12.0, 40.0), true);

  const auto c = calculate_collision(ego, object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.0);
}
}  // namespace autoware::motion_velocity_planner::run_out
