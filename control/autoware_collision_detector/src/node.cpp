// Copyright 2024-2025 TIER IV, Inc.
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

#include "autoware/collision_detector/node.hpp"

#include "autoware/collision_detector/debug.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <collision_detector_node_parameters.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace autoware::collision_detector
{
namespace bg = boost::geometry;
using autoware_utils::create_point;

CollisionDetectorNode::CollisionDetectorNode(const rclcpp::NodeOptions & node_options)
: Node("collision_detector_node", node_options), updater_(this)
{
  param_listener_ =
    std::make_shared<collision_detector_node::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  // Diagnostics Updater
  updater_.setHardwareID("collision_detector");
  updater_.add("collision_detect", this, &CollisionDetectorNode::checkCollision);
  updater_.setPeriod(0.1);

  vehicle_stop_checker_ = std::make_unique<autoware::motion_utils::VehicleStopChecker>(this);
}

tl::expected<PredictedObjects, std::string> CollisionDetectorNode::filterObjects(
  const PredictedObjects & input_objects)
{
  PredictedObjects filtered_objects;
  filtered_objects.header = input_objects.header;

  const rclcpp::Time current_object_time = input_objects.header.stamp;
  const rclcpp::Duration observed_objects_keep_time =
    rclcpp::Duration::from_seconds(0.5);  //  0.5 sec
  const rclcpp::Duration ignored_objects_keep_time =
    rclcpp::Duration::from_seconds(10.0);  // 10 seconds

  // Remove old objects from observed_objects_ and ignored_objects_
  removeOldObjects(observed_objects_, current_object_time, observed_objects_keep_time);
  removeOldObjects(ignored_objects_, current_object_time, ignored_objects_keep_time);

  // Get transform from object frame to base_link
  const auto transform_stamped =
    getTransform("base_link", input_objects.header.frame_id, input_objects.header.stamp, 0.5);

  if (!transform_stamped) {
    return tl::make_unexpected(
      "failed to get transform from " + input_objects.header.frame_id + " to base_link");
  }

  Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped->transform).cast<float>();

  for (const auto & object : input_objects.objects) {
    // Transform object position to base_link frame
    Eigen::Vector3f object_position(
      object.kinematics.initial_pose_with_covariance.pose.position.x,
      object.kinematics.initial_pose_with_covariance.pose.position.y,
      object.kinematics.initial_pose_with_covariance.pose.position.z);
    Eigen::Vector3f transformed_position = isometry * object_position;

    // Calculate object distance from base_link
    const double object_distance = transformed_position.head<2>().norm();
    const bool is_within_range = (object_distance <= params_.nearby_filter_radius);

    // Determine if the object should be excluded based on its classification
    const auto classification =
      object.classification.empty()
        ? autoware_perception_msgs::msg::ObjectClassification::UNKNOWN
        : autoware::object_recognition_utils::getHighestProbLabel(object.classification);
    bool should_be_excluded = shouldBeExcluded(classification);

    const bool is_within_range_and_filtering_class = is_within_range && should_be_excluded;

    // If the object is not within range or not a class to be filtered, add it directly
    if (!is_within_range_and_filtering_class) {
      filtered_objects.objects.push_back(object);

      // Update observed_objects_
      auto observed_it = std::find_if(
        observed_objects_.begin(), observed_objects_.end(),
        [&object](const auto & observed_object) {
          return observed_object.object_id == object.object_id;
        });
      if (observed_it != observed_objects_.end()) {
        observed_it->timestamp = current_object_time;
      } else {
        observed_objects_.push_back({object.object_id, current_object_time});
      }

      continue;
    }

    // Check if the object exists in ignored_objects_
    auto ignored_it = std::find_if(
      ignored_objects_.begin(), ignored_objects_.end(), [&object](const auto & ignored_object) {
        return ignored_object.object_id == object.object_id;
      });
    const bool was_ignored = (ignored_it != ignored_objects_.end());

    // If the object was ignored and is still within the ignore period, continue filtering
    if (
      was_ignored && (current_object_time - ignored_it->timestamp) <
                       rclcpp::Duration::from_seconds(params_.keep_ignoring_time)) {
      // Check if the object exists in observed_objects_
      auto observed_it = std::find_if(
        observed_objects_.begin(), observed_objects_.end(),
        [&object](const auto & observed_object) {
          return observed_object.object_id == object.object_id;
        });
      const bool was_observed = (observed_it != observed_objects_.end());
      if (was_observed) {
        observed_it->timestamp = current_object_time;
      } else {
        // Add as a newly observed object and to the ignore list
        observed_objects_.push_back({object.object_id, current_object_time});
      }
      continue;
    }

    // Check if the object exists in observed_objects_
    auto observed_it = std::find_if(
      observed_objects_.begin(), observed_objects_.end(), [&object](const auto & observed_object) {
        return observed_object.object_id == object.object_id;
      });
    const bool was_observed = (observed_it != observed_objects_.end());

    if (was_observed) {
      observed_it->timestamp = current_object_time;
      // Add without exclusion check
      filtered_objects.objects.push_back(object);
    } else {
      // Add as a newly observed object and to the ignore list
      observed_objects_.push_back({object.object_id, current_object_time});
      ignored_objects_.push_back({object.object_id, current_object_time});
      // Continue filtering
      continue;
    }
  }

  return filtered_objects;
}

void CollisionDetectorNode::removeOldObjects(
  std::vector<TimestampedObject> & container, const rclcpp::Time & current_time,
  const rclcpp::Duration & duration_sec)
{
  container.erase(
    std::remove_if(
      container.begin(), container.end(),
      [&](const TimestampedObject & obj) { return (current_time - obj.timestamp) > duration_sec; }),
    container.end());
}

bool CollisionDetectorNode::shouldBeExcluded(
  const autoware_perception_msgs::msg::ObjectClassification::_label_type & classification) const
{
  switch (classification) {
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
      return params_.nearby_object_type_filters.filter_car;
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
      return params_.nearby_object_type_filters.filter_truck;
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
      return params_.nearby_object_type_filters.filter_bus;
    case autoware_perception_msgs::msg::ObjectClassification::TRAILER:
      return params_.nearby_object_type_filters.filter_trailer;
    case autoware_perception_msgs::msg::ObjectClassification::UNKNOWN:
      return params_.nearby_object_type_filters.filter_unknown;
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      return params_.nearby_object_type_filters.filter_bicycle;
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
      return params_.nearby_object_type_filters.filter_motorcycle;
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      return params_.nearby_object_type_filters.filter_pedestrian;
    case autoware_perception_msgs::msg::ObjectClassification::ANIMAL:
      return params_.nearby_object_type_filters.filter_animal;
    case autoware_perception_msgs::msg::ObjectClassification::HAZARD:
      return params_.nearby_object_type_filters.filter_hazard;
    case autoware_perception_msgs::msg::ObjectClassification::OVER_DRIVABLE:
      return params_.nearby_object_type_filters.filter_over_drivable;
    case autoware_perception_msgs::msg::ObjectClassification::UNDER_DRIVABLE:
      return params_.nearby_object_type_filters.filter_under_drivable;
    default:
      return false;
  }
}

void CollisionDetectorNode::checkCollision(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  odometry_ptr_ = sub_odometry_.take_data();

  if (!odometry_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current odometry...");
    return;
  }

  if (vehicle_stop_checker_->isVehicleStopped()) {
    is_error_diag_ = false;
    start_of_consecutive_collision_stamp_.reset();
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "vehicle is stopping");
    return;
  }

  pointcloud_ptr_ = sub_pointcloud_.take_data();
  object_ptr_ = sub_dynamic_objects_.take_data();
  operation_mode_ptr_ = sub_operation_mode_.take_data();

  if (params_.use_pointcloud && !pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for pointcloud info...");
    return;
  }

  if (params_.use_dynamic_object && !object_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for dynamic object info...");
    return;
  }

  if (!operation_mode_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for operation mode info...");
    return;
  }
  const auto hysteresis = is_error_diag_ ? params_.time_buffer.off_distance_hysteresis : 0.0;
  // The rear overhang is cancelled so that the rear edge sits on the rear axle.
  const auto rear_margin =
    params_.ignore_behind_rear_axle ? vehicle_info_.min_longitudinal_offset_m : hysteresis;
  const auto ego_polygon =
    vehicle_info_.createFootprint(hysteresis, hysteresis, hysteresis, hysteresis, rear_margin);

  auto filtered_objects = filterObjects(*object_ptr_);
  if (!filtered_objects) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "%s",
      filtered_objects.error().c_str());
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, filtered_objects.error());
    return;
  }
  filtered_object_ptr_ = std::make_shared<PredictedObjects>(std::move(*filtered_objects));

  const auto nearest_obstacle = getNearestObstacle(ego_polygon);

  if (!nearest_obstacle && nearest_obstacle.error() == ObstacleSearchError::transform_unavailable) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "failed to get transform to search for obstacles");
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "failed to get transform to search obstacles");
    return;
  }

  const auto is_collision_found =
    nearest_obstacle && nearest_obstacle->first < params_.collision_distance;

  // When a collision is detected, update timestamps to track collision duration
  // - start_of_consecutive_collision_stamp_: marks when a continuous collision began
  // - most_recent_collision_stamp_: records the latest collision detection time
  if (is_collision_found) {
    if (!start_of_consecutive_collision_stamp_.has_value()) {
      start_of_consecutive_collision_stamp_ = this->now();
    }
    most_recent_collision_stamp_ = this->now();
  } else {
    start_of_consecutive_collision_stamp_.reset();
  }

  // Define condition to determine error state based on diagnostic mode
  // 1. When already in error state (is_error_diag_ == true):
  //    - Stay in error if time since last collision is less than off_buffer time
  //    - This creates hysteresis to prevent rapid switching between states
  // 2. When in normal state (is_error_diag_ == false):
  //    - Enter error if collision has been continuous for longer than on_buffer time
  //    - This prevents triggering on brief/momentary collisions
  const auto condition_to_trigger_error = [&]() {
    if (is_error_diag_) {
      return (this->now() - *most_recent_collision_stamp_).seconds() <
             params_.time_buffer.off_duration;
    }
    return start_of_consecutive_collision_stamp_.has_value() &&
           (this->now() - *start_of_consecutive_collision_stamp_).seconds() >=
             params_.time_buffer.on_duration;
  };

  diagnostic_msgs::msg::DiagnosticStatus status;
  if (operation_mode_ptr_->mode == OperationModeState::AUTONOMOUS && condition_to_trigger_error()) {
    is_error_diag_ = true;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "collision detected";
    if (nearest_obstacle) {
      stat.addf("Distance to nearest neighbor object", "%lf", nearest_obstacle->first);
    } else {
      stat.addf(
        "Time since last detection", "%lf",
        (this->now() - *most_recent_collision_stamp_).seconds());
    }
  } else {
    is_error_diag_ = false;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);

  pub_debug_->publish(generate_debug_markers(ego_polygon, nearest_obstacle, is_error_diag_));
}

result_t CollisionDetectorNode::getNearestObstacle(
  const autoware_utils_geometry::LinearRing2d & ego_polygon) const
{
  const auto search_failed = [](const result_t & result) {
    return !result && result.error() == ObstacleSearchError::transform_unavailable;
  };

  const auto closer_of = [&search_failed](const result_t & nearest, const result_t & candidate) {
    if (search_failed(nearest)) {
      return nearest;
    }
    if (search_failed(candidate) || !nearest) {
      return candidate;
    }
    if (!candidate) {
      return nearest;
    }
    return candidate->first < nearest->first ? candidate : nearest;
  };

  result_t nearest_obstacle = tl::make_unexpected(ObstacleSearchError::no_obstacle_found);

  if (params_.use_pointcloud) {
    nearest_obstacle = closer_of(nearest_obstacle, getNearestObstacleByPointCloud(ego_polygon));
  }

  if (params_.use_dynamic_object) {
    nearest_obstacle = closer_of(nearest_obstacle, getNearestObstacleByDynamicObject(ego_polygon));
  }

  return nearest_obstacle;
}

result_t CollisionDetectorNode::getNearestObstacleByPointCloud(
  const autoware_utils_geometry::LinearRing2d & ego_polygon) const
{
  const auto transform_stamped =
    getTransform("base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::infinity();

  if (!transform_stamped) {
    return tl::make_unexpected(ObstacleSearchError::transform_unavailable);
  }

  const Eigen::Affine3f isometry =
    tf2::transformToEigen(transform_stamped->transform).cast<float>();

  // The bounding box rejects far points before the exact polygon distance is calculated.
  const auto ego_box = bg::return_envelope<autoware_utils_geometry::Box2d>(ego_polygon);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointcloud_ptr_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointcloud_ptr_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointcloud_ptr_, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const Eigen::Vector3f point = isometry * Eigen::Vector3f(*iter_x, *iter_y, *iter_z);

    // An infinite minimum distance widens the box to infinity, so no point is rejected yet.
    const bool is_outside_of_box =
      point.x() < bg::get<bg::min_corner, 0>(ego_box) - minimum_distance ||
      point.x() > bg::get<bg::max_corner, 0>(ego_box) + minimum_distance ||
      point.y() < bg::get<bg::min_corner, 1>(ego_box) - minimum_distance ||
      point.y() > bg::get<bg::max_corner, 1>(ego_box) + minimum_distance;
    if (is_outside_of_box) {
      continue;
    }

    const autoware_utils_geometry::Point2d boost_point(point.x(), point.y());
    const auto distance_to_object = bg::distance(ego_polygon, boost_point);

    if (distance_to_object < minimum_distance) {
      nearest_point = create_point(point.x(), point.y(), point.z());
      minimum_distance = distance_to_object;
    }

    // No point can be closer than a point inside the polygon, so the search is finished.
    if (minimum_distance <= 0.0) {
      break;
    }
  }

  if (!std::isfinite(minimum_distance)) {
    return tl::make_unexpected(ObstacleSearchError::no_obstacle_found);
  }

  return std::make_pair(minimum_distance, nearest_point);
}

result_t CollisionDetectorNode::getNearestObstacleByDynamicObject(
  const autoware_utils_geometry::LinearRing2d & ego_polygon) const
{
  const auto transform_stamped = getTransform(
    filtered_object_ptr_->header.frame_id, "base_link", filtered_object_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::infinity();

  if (!transform_stamped) {
    return tl::make_unexpected(ObstacleSearchError::transform_unavailable);
  }

  tf2::Transform tf_src2target;
  tf2::fromMsg(transform_stamped->transform, tf_src2target);
  const auto tf_target2src = tf_src2target.inverse();

  for (const auto & object : filtered_object_ptr_->objects) {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;

    tf2::Transform tf_src2object;
    tf2::fromMsg(object_pose, tf_src2object);

    geometry_msgs::msg::Pose transformed_object_pose;
    tf2::toMsg(tf_target2src * tf_src2object, transformed_object_pose);

    // to_polygon2d throws on an unknown type, so the shape falls back to a bounding box.
    auto shape = object.shape;
    if (
      shape.type != Shape::POLYGON && shape.type != Shape::CYLINDER &&
      shape.type != Shape::BOUNDING_BOX) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 5000 /* ms */, "Unsupported shape type: %d", shape.type);
      shape.type = Shape::BOUNDING_BOX;
    }

    const auto object_polygon =
      autoware_utils_geometry::to_polygon2d(transformed_object_pose, shape);

    const auto distance_to_object = bg::distance(ego_polygon, object_polygon);

    if (distance_to_object < minimum_distance) {
      nearest_point = object_pose.position;
      minimum_distance = distance_to_object;
    }
  }

  if (!std::isfinite(minimum_distance)) {
    return tl::make_unexpected(ObstacleSearchError::no_obstacle_found);
  }

  return std::make_pair(minimum_distance, nearest_point);
}

std::optional<geometry_msgs::msg::TransformStamped> CollisionDetectorNode::getTransform(
  const std::string & source, const std::string & target, const rclcpp::Time & stamp,
  double duration_sec) const
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped =
      tf_buffer_.lookupTransform(source, target, stamp, tf2::durationFromSec(duration_sec));
  } catch (const tf2::TransformException & ex) {
    return {};
  }

  return transform_stamped;
}

}  // namespace autoware::collision_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::collision_detector::CollisionDetectorNode)
