//
//  Copyright 2025 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef BAG_TIME_MANAGER_PANEL_HPP_
#define BAG_TIME_MANAGER_PANEL_HPP_

#include <qt5/QtCore/QMetaObject>
#include <qt5/QtCore/QProcess>
#include <qt5/QtCore/QTimer>
#include <qt5/QtWidgets/QCheckBox>
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QLineEdit>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QSlider>
#include <qt5/QtWidgets/QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_interfaces/srv/pause.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>
#include <rosbag2_interfaces/srv/set_rate.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace autoware::visualization::bag_time_manager_rviz_plugin
{
using rosbag2_interfaces::srv::Pause;
using rosbag2_interfaces::srv::Resume;
using rosbag2_interfaces::srv::SetRate;
class BagTimeManagerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit BagTimeManagerPanel(QWidget * parent = nullptr);
  ~BagTimeManagerPanel() override;
  void onInitialize() override;

protected Q_SLOTS:
  /// @brief callback for when the publishing rate is changed
  void onRateChanged() {}
  /// @brief callback for when the step button is clicked
  void onPauseClicked();
  void onApplyRateClicked();
  /// @brief callback for when the browse-file button is clicked
  void onBrowseFileClicked();
  /// @brief callback for when the browse-folder button is clicked
  void onBrowseFolderClicked();
  /// @brief callback for when the play button is clicked
  void onPlayClicked();
  /// @brief callback for when the start-time trim slider is dragged
  void onStartTimeSliderChanged(int value);
  /// @brief callback for when the end-time trim slider is dragged
  void onEndTimeSliderChanged(int value);
  /// @brief callback for when the publish-/clock checkbox is toggled
  void onClockCheckBoxToggled(bool checked);
  /// @brief callback for when the stop button is clicked
  void onStopClicked();
  /// @brief callback for when the running rosbag2 player process exits
  void onBagProcessFinished(int exit_code, QProcess::ExitStatus exit_status);
  /// @brief callback for when the rosbag2 player process fails to start
  void onBagProcessErrorOccurred(QProcess::ProcessError error);
  /// @brief callback that refreshes the elapsed/remaining bag time label
  void onTimeUpdateTimeout();
  /// @brief callback for when the save-route button is clicked
  void onSaveRouteClicked();
  /// @brief callback for when the load-route button is clicked
  void onLoadRouteClicked();
  /// @brief callback for when the publish-route button is clicked
  void onPublishRouteClicked();
  /// @brief refreshes the route status label and enables save/publish; safe to call from the Qt
  /// thread only (invoked with a queued connection from the route subscription callback, which
  /// runs on the ROS executor thread)
  void onRouteCaptured();

protected:
  // ROS
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Client<Pause>::SharedPtr client_pause_;
  rclcpp::Client<Resume>::SharedPtr client_resume_;
  rclcpp::Client<SetRate>::SharedPtr client_set_rate_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_subscription_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_publisher_;

  // GUI
  QLineEdit * bag_path_edit_;
  QPushButton * browse_file_button_;
  QPushButton * browse_folder_button_;
  QCheckBox * clock_checkbox_;
  QSpinBox * clock_rate_spinbox_;
  QPushButton * play_button_;
  QPushButton * stop_button_;
  QPushButton * pause_button_;
  QPushButton * apply_rate_button_;
  QLabel * rate_label_;
  QLabel * time_label_;
  QComboBox * rate_combo_;
  QLabel * bag_time_label_;
  QSlider * start_time_slider_;
  QLabel * start_time_value_label_;
  QSlider * end_time_slider_;
  QLabel * end_time_value_label_;
  QLabel * route_status_label_;
  QPushButton * save_route_button_;
  QPushButton * load_route_button_;
  QPushButton * publish_route_button_;

private:
  enum STATE { PAUSE, RESUME };
  STATE current_state_{RESUME};

  /// @brief terminate the currently running rosbag2 player process, if any
  void stopBagProcess();
  /// @brief run `ros2 bag info` on the given path to read its start time and duration
  static bool queryBagInfo(const QString & bag_path, double & start_sec, double & duration_sec);
  /// @brief reset the elapsed/remaining time tracking and label back to idle
  void resetTimeTracking();
  /// @brief look up the given path's duration and (re)configure the trim sliders for it
  void updateTrimRangeForPath(const QString & bag_path);

  QProcess * bag_process_{nullptr};
  QTimer * time_update_timer_;
  std::atomic<int64_t> latest_clock_ns_{0};
  double bag_start_sec_{0.0};
  double bag_duration_sec_{0.0};
  bool has_bag_start_{false};
  bool end_trim_active_{false};

  std::mutex route_mutex_;
  autoware_planning_msgs::msg::LaneletRoute::SharedPtr captured_route_;
};

}  // namespace autoware::visualization::bag_time_manager_rviz_plugin

#endif  // BAG_TIME_MANAGER_PANEL_HPP_
