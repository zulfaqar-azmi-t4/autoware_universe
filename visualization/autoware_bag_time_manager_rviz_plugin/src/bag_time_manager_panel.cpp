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

#include "bag_time_manager_panel.hpp"

#include <qt5/QtCore/QFile>
#include <qt5/QtCore/QRegularExpression>
#include <qt5/QtWidgets/QFileDialog>
#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QVBoxLayout>
#include <qt5/QtWidgets/QWidget>
#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>

namespace autoware::visualization::bag_time_manager_rviz_plugin
{
BagTimeManagerPanel::BagTimeManagerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // bag selection / playback
  {
    bag_path_edit_ = new QLineEdit();
    bag_path_edit_->setPlaceholderText("Select a rosbag file or folder...");
    browse_file_button_ = new QPushButton("Browse File...");
    browse_file_button_->setToolTip("Select a rosbag storage file (.db3 or .mcap).");
    browse_folder_button_ = new QPushButton("Browse Folder...");
    browse_folder_button_->setToolTip("Select a rosbag folder (containing metadata.yaml).");
    clock_checkbox_ = new QCheckBox("Publish /clock");
    clock_checkbox_->setToolTip("Publish simulated time on /clock while playing.");
    clock_checkbox_->setChecked(true);
    clock_rate_spinbox_ = new QSpinBox();
    clock_rate_spinbox_->setSuffix(" Hz");
    clock_rate_spinbox_->setRange(1, 1000);
    clock_rate_spinbox_->setValue(200);
    clock_rate_spinbox_->setToolTip("Frequency to publish /clock at.");
    play_button_ = new QPushButton("Play");
    play_button_->setToolTip("Play the selected rosbag.");
    stop_button_ = new QPushButton("Stop");
    stop_button_->setToolTip("Stop the running rosbag2 player.");

    bag_time_label_ = new QLabel("Elapsed: -- / Remaining: --");
    bag_time_label_->setToolTip(
      "Elapsed/remaining bag playback time, derived from /clock and `ros2 bag info`.");

    time_update_timer_ = new QTimer(this);
    time_update_timer_->setInterval(200);

    start_time_slider_ = new QSlider(Qt::Horizontal);
    start_time_slider_->setToolTip("Start playback this far into the bag.");
    start_time_slider_->setEnabled(false);
    start_time_value_label_ = new QLabel("--");
    end_time_slider_ = new QSlider(Qt::Horizontal);
    end_time_slider_->setToolTip("Stop playback at this point in the bag.");
    end_time_slider_->setEnabled(false);
    end_time_value_label_ = new QLabel("--");
  }

  // route capture / replay
  {
    route_status_label_ = new QLabel("Route: no route captured yet");
    route_status_label_->setToolTip(
      "Status of the captured/loaded /planning/mission_planning/route message.");
    save_route_button_ = new QPushButton("Save Route...");
    save_route_button_->setToolTip("Save the captured/loaded route to a file.");
    save_route_button_->setEnabled(false);
    load_route_button_ = new QPushButton("Load Route...");
    load_route_button_->setToolTip("Load a previously saved route from a file.");
    publish_route_button_ = new QPushButton("Publish Route");
    publish_route_button_->setToolTip(
      "Publish the captured/loaded route to /planning/mission_planning/route.");
    publish_route_button_->setEnabled(false);
  }

  // pause / resume
  {
    pause_button_ = new QPushButton("Pause");
    pause_button_->setToolTip("Pause/Resume ROS time.");
    pause_button_->setStyleSheet("background-color: #00FF00;");
    pause_button_->setCheckable(true);
  }

  // apply
  {
    apply_rate_button_ = new QPushButton("ApplyRate");
    apply_rate_button_->setToolTip("control ROS time rate.");
  }

  // combo
  {
    rate_label_ = new QLabel("Rate:");
    rate_label_->setAlignment(Qt::AlignCenter);
    rate_combo_ = new QComboBox();
    rate_combo_->addItems({"0.01", "0.1", "0.5", "1.0", "2.0", "5.0", "10.0"});
    rate_combo_->setCurrentText(QString("1.0"));
    time_label_ = new QLabel("X  real time ");
    rate_label_->setAlignment(Qt::AlignCenter);
  }

  auto * bag_layout = new QHBoxLayout();
  bag_layout->addWidget(bag_path_edit_);
  bag_layout->addWidget(browse_file_button_);
  bag_layout->addWidget(browse_folder_button_);

  auto * playback_layout = new QHBoxLayout();
  playback_layout->addWidget(clock_checkbox_);
  playback_layout->addWidget(clock_rate_spinbox_);
  playback_layout->addWidget(play_button_);
  playback_layout->addWidget(stop_button_);

  auto * start_trim_layout = new QHBoxLayout();
  start_trim_layout->addWidget(new QLabel("Start:"));
  start_trim_layout->addWidget(start_time_slider_);
  start_trim_layout->addWidget(start_time_value_label_);

  auto * end_trim_layout = new QHBoxLayout();
  end_trim_layout->addWidget(new QLabel("End:"));
  end_trim_layout->addWidget(end_time_slider_);
  end_trim_layout->addWidget(end_time_value_label_);

  auto * status_layout = new QHBoxLayout();
  status_layout->addWidget(bag_time_label_);

  auto * route_layout = new QHBoxLayout();
  route_layout->addWidget(route_status_label_);
  route_layout->addWidget(save_route_button_);
  route_layout->addWidget(load_route_button_);
  route_layout->addWidget(publish_route_button_);

  auto * control_layout = new QHBoxLayout();
  control_layout->addWidget(pause_button_);
  control_layout->addWidget(apply_rate_button_);
  control_layout->addWidget(rate_label_);
  control_layout->addWidget(rate_combo_);
  control_layout->addWidget(time_label_);

  auto * layout = new QVBoxLayout();
  layout->addLayout(bag_layout);
  layout->addLayout(playback_layout);
  layout->addLayout(start_trim_layout);
  layout->addLayout(end_trim_layout);
  layout->addLayout(status_layout);
  layout->addLayout(route_layout);
  layout->addLayout(control_layout);
  setLayout(layout);

  connect(browse_file_button_, SIGNAL(clicked()), this, SLOT(onBrowseFileClicked()));
  connect(browse_folder_button_, SIGNAL(clicked()), this, SLOT(onBrowseFolderClicked()));
  connect(play_button_, SIGNAL(clicked()), this, SLOT(onPlayClicked()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopClicked()));
  connect(clock_checkbox_, SIGNAL(toggled(bool)), this, SLOT(onClockCheckBoxToggled(bool)));
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseClicked()));
  connect(apply_rate_button_, SIGNAL(clicked()), this, SLOT(onApplyRateClicked()));
  connect(rate_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(onRateChanged()));
  connect(time_update_timer_, SIGNAL(timeout()), this, SLOT(onTimeUpdateTimeout()));
  connect(start_time_slider_, SIGNAL(valueChanged(int)), this, SLOT(onStartTimeSliderChanged(int)));
  connect(end_time_slider_, SIGNAL(valueChanged(int)), this, SLOT(onEndTimeSliderChanged(int)));
  connect(save_route_button_, SIGNAL(clicked()), this, SLOT(onSaveRouteClicked()));
  connect(load_route_button_, SIGNAL(clicked()), this, SLOT(onLoadRouteClicked()));
  connect(publish_route_button_, SIGNAL(clicked()), this, SLOT(onPublishRouteClicked()));
}

BagTimeManagerPanel::~BagTimeManagerPanel()
{
  stopBagProcess();
}

void BagTimeManagerPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  client_pause_ = raw_node_->create_client<Pause>("/rosbag2_player/pause");
  client_resume_ = raw_node_->create_client<Resume>("/rosbag2_player/resume");
  client_set_rate_ = raw_node_->create_client<SetRate>("/rosbag2_player/set_rate");

  // mission_planner publishes the route with QoS(1).transient_local(); match it so the
  // subscription also picks up a route that was published before this panel started.
  const auto route_qos = rclcpp::QoS(1).transient_local();
  route_subscription_ = raw_node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "/planning/mission_planning/route", route_qos,
    [this](const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(route_mutex_);
        captured_route_ = msg;
      }
      QMetaObject::invokeMethod(this, "onRouteCaptured", Qt::QueuedConnection);
    });
  route_publisher_ = raw_node_->create_publisher<autoware_planning_msgs::msg::LaneletRoute>(
    "/planning/mission_planning/route", route_qos);
}

void BagTimeManagerPanel::onBrowseFileClicked()
{
  const auto bag_path = QFileDialog::getOpenFileName(
    this, "Select Rosbag File", QString(), "Rosbag files (*.db3 *.mcap);;All files (*)");
  if (bag_path.isEmpty()) {
    return;
  }
  bag_path_edit_->setText(bag_path);
  updateTrimRangeForPath(bag_path);
}

void BagTimeManagerPanel::onBrowseFolderClicked()
{
  const auto bag_path = QFileDialog::getExistingDirectory(this, "Select Rosbag Folder");
  if (bag_path.isEmpty()) {
    return;
  }
  bag_path_edit_->setText(bag_path);
  updateTrimRangeForPath(bag_path);
}

void BagTimeManagerPanel::onClockCheckBoxToggled(bool checked)
{
  clock_rate_spinbox_->setEnabled(checked);
}

void BagTimeManagerPanel::onStartTimeSliderChanged(int value)
{
  if (value > end_time_slider_->value()) {
    end_time_slider_->setValue(value);
  }
  start_time_value_label_->setText(QString("%1s").arg(value / 10.0, 0, 'f', 1));
}

void BagTimeManagerPanel::onEndTimeSliderChanged(int value)
{
  if (value < start_time_slider_->value()) {
    start_time_slider_->setValue(value);
  }
  end_time_value_label_->setText(QString("%1s").arg(value / 10.0, 0, 'f', 1));
}

void BagTimeManagerPanel::updateTrimRangeForPath(const QString & bag_path)
{
  double start_sec = 0.0;
  double duration_sec = 0.0;
  if (!queryBagInfo(bag_path, start_sec, duration_sec) || duration_sec <= 0.0) {
    start_time_slider_->setEnabled(false);
    end_time_slider_->setEnabled(false);
    start_time_slider_->setRange(0, 0);
    end_time_slider_->setRange(0, 0);
    start_time_value_label_->setText("--");
    end_time_value_label_->setText("--");
    return;
  }

  // 0.1s resolution.
  const int max_tenths = static_cast<int>(std::round(duration_sec * 10.0));
  start_time_slider_->setEnabled(true);
  end_time_slider_->setEnabled(true);
  start_time_slider_->setRange(0, max_tenths);
  end_time_slider_->setRange(0, max_tenths);
  start_time_slider_->setValue(0);
  end_time_slider_->setValue(max_tenths);
  start_time_value_label_->setText(QString("%1s").arg(0.0, 0, 'f', 1));
  end_time_value_label_->setText(QString("%1s").arg(duration_sec, 0, 'f', 1));
}

void BagTimeManagerPanel::onPlayClicked()
{
  const auto bag_path = bag_path_edit_->text();
  if (bag_path.isEmpty()) {
    RCLCPP_WARN(raw_node_->get_logger(), "no rosbag file or folder selected");
    return;
  }

  stopBagProcess();

  double bag_start_epoch_sec = 0.0;
  double bag_duration_sec = 0.0;
  const bool has_bag_info = queryBagInfo(bag_path, bag_start_epoch_sec, bag_duration_sec);
  if (!has_bag_info) {
    RCLCPP_WARN(
      raw_node_->get_logger(),
      "failed to read `ros2 bag info` for %s; elapsed/remaining time "
      "will not be shown and start/end trim will be ignored",
      bag_path.toStdString().c_str());
  }

  QStringList args{"bag", "play", bag_path};

  double start_offset_sec = 0.0;
  double end_offset_sec = bag_duration_sec;
  const bool trim_enabled = has_bag_info && start_time_slider_->isEnabled();
  const bool publish_clock = clock_checkbox_->isChecked();
  end_trim_active_ = false;
  if (trim_enabled) {
    start_offset_sec = start_time_slider_->value() / 10.0;
    end_offset_sec = end_time_slider_->value() / 10.0;
    if (start_offset_sec > 0.0) {
      args << "--start-offset" << QString::number(start_offset_sec, 'f', 2);
    }
    if (end_offset_sec < bag_duration_sec - 1e-6) {
      // `ros2 bag play` has no "stop at timestamp" flag on this ROS distro, so the end trim is
      // enforced by this panel: onTimeUpdateTimeout() stops the process once elapsed /clock time
      // reaches the trimmed duration. That requires /clock to be published.
      if (publish_clock) {
        end_trim_active_ = true;
      } else {
        RCLCPP_WARN(
          raw_node_->get_logger(),
          "end trim requires \"Publish /clock\" to be enabled; playing to the end of the bag");
      }
    }
  }

  has_bag_start_ = has_bag_info;
  bag_start_sec_ = bag_start_epoch_sec + start_offset_sec;
  bag_duration_sec_ = has_bag_info ? std::max(0.0, end_offset_sec - start_offset_sec) : 0.0;
  latest_clock_ns_.store(static_cast<int64_t>(bag_start_sec_ * 1e9), std::memory_order_relaxed);

  bag_process_ = new QProcess(this);
  connect(
    bag_process_, SIGNAL(finished(int, QProcess::ExitStatus)), this,
    SLOT(onBagProcessFinished(int, QProcess::ExitStatus)));
  connect(
    bag_process_, SIGNAL(errorOccurred(QProcess::ProcessError)), this,
    SLOT(onBagProcessErrorOccurred(QProcess::ProcessError)));

  if (publish_clock) {
    // Publishes /clock so downstream nodes using sim time stay in sync with the bag.
    args << "--clock" << QString::number(clock_rate_spinbox_->value());
  }
  bag_process_->start("ros2", args);

  if (publish_clock) {
    // rosbag2 player publishes /clock with ClockQoS (best-effort); a default reliable
    // subscription would never match it and elapsed time would never advance.
    clock_subscription_ = raw_node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(), [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
        const int64_t stamp_ns =
          static_cast<int64_t>(msg->clock.sec) * 1000000000LL + msg->clock.nanosec;
        latest_clock_ns_.store(stamp_ns, std::memory_order_relaxed);
      });
    time_update_timer_->start();
  } else if (has_bag_start_) {
    bag_time_label_->setText(QString("Duration: %1s (enable Publish /clock for elapsed time)")
                               .arg(bag_duration_sec_, 0, 'f', 1));
  }

  current_state_ = STATE::RESUME;
  pause_button_->setChecked(false);
  pause_button_->setText(QString::fromStdString("Pause"));
  pause_button_->setStyleSheet("background-color: #00FF00;");
}

void BagTimeManagerPanel::onStopClicked()
{
  stopBagProcess();
}

void BagTimeManagerPanel::onBagProcessFinished(int exit_code, QProcess::ExitStatus exit_status)
{
  if (exit_code != 0 || exit_status == QProcess::CrashExit) {
    const auto stderr_output = QString::fromUtf8(bag_process_->readAllStandardError()).trimmed();
    RCLCPP_WARN(
      raw_node_->get_logger(), "rosbag2 player process exited with code %d: %s", exit_code,
      stderr_output.isEmpty() ? "no error output" : stderr_output.toStdString().c_str());
  } else {
    RCLCPP_INFO(raw_node_->get_logger(), "rosbag2 player process exited with code %d", exit_code);
  }
  resetTimeTracking();
}

void BagTimeManagerPanel::onBagProcessErrorOccurred([[maybe_unused]] QProcess::ProcessError error)
{
  RCLCPP_WARN(
    raw_node_->get_logger(), "failed to run rosbag2 player: %s",
    bag_process_->errorString().toStdString().c_str());
  resetTimeTracking();
}

bool BagTimeManagerPanel::queryBagInfo(
  const QString & bag_path, double & start_sec, double & duration_sec)
{
  QProcess info_process;
  info_process.start("ros2", {"bag", "info", bag_path});
  if (!info_process.waitForFinished(5000)) {
    return false;
  }

  const auto output = QString::fromUtf8(info_process.readAllStandardOutput());
  static const QRegularExpression duration_pattern(R"(Duration:\s*([0-9.]+)s)");
  static const QRegularExpression start_pattern(R"(Start:.*\(([0-9.]+)\))");

  const auto duration_match = duration_pattern.match(output);
  const auto start_match = start_pattern.match(output);
  if (!duration_match.hasMatch() || !start_match.hasMatch()) {
    return false;
  }

  duration_sec = duration_match.captured(1).toDouble();
  start_sec = start_match.captured(1).toDouble();
  return true;
}

void BagTimeManagerPanel::onTimeUpdateTimeout()
{
  if (!has_bag_start_) {
    return;
  }

  const double current_sec =
    static_cast<double>(latest_clock_ns_.load(std::memory_order_relaxed)) / 1e9;
  const double elapsed_sec = std::max(0.0, current_sec - bag_start_sec_);

  if (end_trim_active_ && elapsed_sec >= bag_duration_sec_) {
    RCLCPP_INFO(raw_node_->get_logger(), "reached trimmed end of bag playback, stopping");
    stopBagProcess();
    return;
  }

  if (bag_duration_sec_ > 0.0) {
    const double remaining_sec = std::max(0.0, bag_duration_sec_ - elapsed_sec);
    bag_time_label_->setText(QString("Elapsed: %1s / %2s (remaining %3s)")
                               .arg(elapsed_sec, 0, 'f', 1)
                               .arg(bag_duration_sec_, 0, 'f', 1)
                               .arg(remaining_sec, 0, 'f', 1));
  } else {
    bag_time_label_->setText(QString("Elapsed: %1s").arg(elapsed_sec, 0, 'f', 1));
  }
}

void BagTimeManagerPanel::resetTimeTracking()
{
  has_bag_start_ = false;
  end_trim_active_ = false;
  bag_start_sec_ = 0.0;
  bag_duration_sec_ = 0.0;
  latest_clock_ns_.store(0, std::memory_order_relaxed);
  time_update_timer_->stop();
  clock_subscription_.reset();
  bag_time_label_->setText("Elapsed: -- / Remaining: --");
}

void BagTimeManagerPanel::stopBagProcess()
{
  resetTimeTracking();
  if (bag_process_ == nullptr) {
    return;
  }
  // Disconnect first: a termination we requested ourselves is not a playback failure.
  disconnect(bag_process_, nullptr, this, nullptr);
  if (bag_process_->state() != QProcess::NotRunning) {
    bag_process_->terminate();
    if (!bag_process_->waitForFinished(2000)) {
      bag_process_->kill();
      bag_process_->waitForFinished();
    }
  }
  bag_process_->deleteLater();
  bag_process_ = nullptr;
}

void BagTimeManagerPanel::onPauseClicked()
{
  if (current_state_ == STATE::PAUSE) {
    // do resume
    current_state_ = STATE::RESUME;
    pause_button_->setText(QString::fromStdString("Resume"));
    // green
    pause_button_->setStyleSheet("background-color: #00FF00;");
    auto req = std::make_shared<Resume::Request>();
    client_resume_->async_send_request(
      req, []([[maybe_unused]] rclcpp::Client<Resume>::SharedFuture result) {});
  } else {
    // do pause
    current_state_ = STATE::PAUSE;
    pause_button_->setText(QString::fromStdString("Pause"));
    // red
    pause_button_->setStyleSheet("background-color: #FF0000;");
    auto req = std::make_shared<Pause::Request>();
    client_pause_->async_send_request(
      req, []([[maybe_unused]] rclcpp::Client<Pause>::SharedFuture result) {});
  }
}

void BagTimeManagerPanel::onApplyRateClicked()
{
  auto request = std::make_shared<SetRate::Request>();
  request->rate = std::stod(rate_combo_->currentText().toStdString());
  client_set_rate_->async_send_request(
    request, [this, request](rclcpp::Client<SetRate>::SharedFuture result) {
      const auto & response = result.get();
      if (response->success) {
        RCLCPP_INFO(raw_node_->get_logger(), "set ros bag rate %f x real time", request->rate);
      } else {
        RCLCPP_WARN(raw_node_->get_logger(), "service failed");
      }
    });
}

void BagTimeManagerPanel::onRouteCaptured()
{
  std::lock_guard<std::mutex> lock(route_mutex_);
  if (!captured_route_) {
    return;
  }
  route_status_label_->setText(
    QString("Route: ready to publish (%1 segments)").arg(captured_route_->segments.size()));
  save_route_button_->setEnabled(true);
  publish_route_button_->setEnabled(true);
}

void BagTimeManagerPanel::onSaveRouteClicked()
{
  autoware_planning_msgs::msg::LaneletRoute::SharedPtr route_to_save;
  {
    std::lock_guard<std::mutex> lock(route_mutex_);
    route_to_save = captured_route_;
  }
  if (!route_to_save) {
    RCLCPP_WARN(raw_node_->get_logger(), "no route captured or loaded yet");
    return;
  }

  const auto file_path = QFileDialog::getSaveFileName(
    this, "Save Route", QString(), "Route files (*.route);;All files (*)");
  if (file_path.isEmpty()) {
    return;
  }

  rclcpp::Serialization<autoware_planning_msgs::msg::LaneletRoute> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(route_to_save.get(), &serialized_msg);

  QFile file(file_path);
  if (!file.open(QIODevice::WriteOnly)) {
    RCLCPP_WARN(
      raw_node_->get_logger(), "failed to open %s for writing", file_path.toStdString().c_str());
    return;
  }
  file.write(
    reinterpret_cast<const char *>(serialized_msg.get_rcl_serialized_message().buffer),
    static_cast<qint64>(serialized_msg.size()));
  file.close();
  RCLCPP_INFO(raw_node_->get_logger(), "saved route to %s", file_path.toStdString().c_str());
}

void BagTimeManagerPanel::onLoadRouteClicked()
{
  const auto file_path = QFileDialog::getOpenFileName(
    this, "Load Route", QString(), "Route files (*.route);;All files (*)");
  if (file_path.isEmpty()) {
    return;
  }

  QFile file(file_path);
  if (!file.open(QIODevice::ReadOnly)) {
    RCLCPP_WARN(
      raw_node_->get_logger(), "failed to open %s for reading", file_path.toStdString().c_str());
    return;
  }
  const QByteArray data = file.readAll();
  file.close();

  rclcpp::SerializedMessage serialized_msg(static_cast<size_t>(data.size()));
  auto & rcl_serialized_msg = serialized_msg.get_rcl_serialized_message();
  std::memcpy(rcl_serialized_msg.buffer, data.constData(), static_cast<size_t>(data.size()));
  rcl_serialized_msg.buffer_length = static_cast<size_t>(data.size());

  auto loaded_route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
  rclcpp::Serialization<autoware_planning_msgs::msg::LaneletRoute> serializer;
  serializer.deserialize_message(&serialized_msg, loaded_route.get());

  {
    std::lock_guard<std::mutex> lock(route_mutex_);
    captured_route_ = loaded_route;
  }
  onRouteCaptured();
  RCLCPP_INFO(raw_node_->get_logger(), "loaded route from %s", file_path.toStdString().c_str());
}

void BagTimeManagerPanel::onPublishRouteClicked()
{
  autoware_planning_msgs::msg::LaneletRoute::SharedPtr route_to_publish;
  {
    std::lock_guard<std::mutex> lock(route_mutex_);
    route_to_publish = captured_route_;
  }
  if (!route_to_publish) {
    RCLCPP_WARN(raw_node_->get_logger(), "no route captured or loaded yet");
    return;
  }
  route_publisher_->publish(*route_to_publish);
  RCLCPP_INFO(raw_node_->get_logger(), "published route to %s", route_publisher_->get_topic_name());
}
}  // namespace autoware::visualization::bag_time_manager_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::visualization::bag_time_manager_rviz_plugin::BagTimeManagerPanel, rviz_common::Panel)
