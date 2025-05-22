#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/control_command.hpp"
#include "hexapod_msgs/msg/leg_pose.hpp"
#include "hexapod_msgs/msg/leg_poses.hpp"
#include "hexapod_rviz_plugins/control_panel.hpp"
#include <QApplication>
#include <cstddef>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <qspinbox.h>
#include <rclcpp/context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <string>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {

HexapodControlRvizPanel::HexapodControlRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  setupUi();
}

HexapodControlRvizPanel::~HexapodControlRvizPanel() {}

void HexapodControlRvizPanel::onInitialize() {
  DisplayContext *context = getDisplayContext();
  auto ros_node_abstraction_weak = context->getRosNodeAbstraction();
  auto ros_node_abstraction = ros_node_abstraction_weak.lock();
  if (!context) {
    throw std::runtime_error("RViz context not available");
  }
  node_ = ros_node_abstraction->get_raw_node();
  setupROS();
}

void HexapodControlRvizPanel::setupUi() {
  QVBoxLayout *main_layout = new QVBoxLayout;

  // Create main horizontal layout
  QHBoxLayout *horizontal_layout = new QHBoxLayout;

  // Left side - leg position table
  leg_table_ = new QTableWidget(6, 4); // 6 legs, 4 columns (name + xyz)
  leg_table_->setHorizontalHeaderLabels({"Leg", "X", "Y", "Z"});
  // leg_table_->verticalHeader()->setVisible(false);

  // Set leg names
  const QStringList leg_names = {"top_left",  "mid_left",  "bottom_left",
                                 "top_right", "mid_right", "bottom_right"};
  for (int i = 0; i < 6; ++i) {
    // Leg name
    QTableWidgetItem *name_item = new QTableWidgetItem(leg_names[i]);
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    leg_table_->setItem(i, 0, name_item);

    std::array<QDoubleSpinBox *, 3> spin_array;
    // X, Y, Z spin boxes
    for (int j = 0; j < 3; ++j) {
      QWidget *widget = new QWidget();
      QHBoxLayout *layout = new QHBoxLayout(widget);
      layout->setContentsMargins(0, 0, 0, 0);

      QDoubleSpinBox *spin_box = createSpinBox();
      layout->addWidget(spin_box);
      widget->setLayout(layout);
      spin_box->setProperty("legName", leg_names[i]);
      spin_box->setProperty("axis", j); // 0 = X, 1 = Y, 2 = Z

      leg_table_->setCellWidget(i, j + 1, widget);

      // Connect value changes
      connect(spin_box, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, &HexapodControlRvizPanel::onSpinnerBoxUpdate);
      spin_array[j] = spin_box;
    }

    spinners_[leg_names[i].toStdString()] = spin_array;
  }
  leg_table_->resizeColumnsToContents();
  horizontal_layout->addWidget(leg_table_, 1); // Stretch factor 1 for table

  // Right side - controls and pose list
  QVBoxLayout *right_layout = new QVBoxLayout();

  // Pose navigation controls with icons
  QHBoxLayout *nav_layout = new QHBoxLayout;
  QPushButton *prev_button = new QPushButton();
  prev_button->setIcon(QApplication::style()->standardIcon(QStyle::SP_ArrowUp));
  prev_button->setToolTip("Previous pose");
  QPushButton *next_button = new QPushButton();
  next_button->setIcon(
      QApplication::style()->standardIcon(QStyle::SP_ArrowDown));
  next_button->setToolTip("Next pose");
  nav_layout->addWidget(prev_button);
  nav_layout->addWidget(next_button);
  right_layout->addLayout(nav_layout);

  connect(prev_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onPreviousPoseClicked);
  connect(next_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onNextPoseClicked);

  // Pose saving controls
  QHBoxLayout *save_layout = new QHBoxLayout;
  pose_name_input_ = new QLineEdit();
  pose_name_input_->setPlaceholderText("Pose name");
  QPushButton *save_button = new QPushButton("Save Pose");
  QPushButton *delete_button = new QPushButton("Delete Pose");
  save_layout->addWidget(pose_name_input_);
  save_layout->addWidget(save_button);
  save_layout->addWidget(delete_button);
  right_layout->addLayout(save_layout);

  connect(save_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onSavePoseClicked);
  connect(delete_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onDeletePoseClicked);

  // Pose list
  right_layout->addWidget(new QLabel("Saved Poses:"));
  pose_list_ = new QListWidget();
  right_layout->addWidget(pose_list_, 2); // Stretch factor 1 for list
  connect(pose_list_, &QListWidget::itemClicked, this,
          &HexapodControlRvizPanel::onPoseSelected);

  horizontal_layout->addLayout(right_layout,
                               1); // Stretch factor 1 for right side
  main_layout->addLayout(horizontal_layout);

  setLayout(main_layout);
}

QDoubleSpinBox *HexapodControlRvizPanel::createSpinBox() {
  QDoubleSpinBox *spin_box = new QDoubleSpinBox();
  spin_box->setRange(-100.0, 100.0);
  spin_box->setSingleStep(0.005);
  spin_box->setDecimals(3);
  spin_box->setStyleSheet("QDoubleSpinBox { background: transparent; }");
  return spin_box;
}

void HexapodControlRvizPanel::setupROS() {
  leg_pose_pub_ = node_->create_publisher<hexapod_msgs::msg::LegPose>(
      "hexapod_control/leg_pose/update", 10);

  command_pub_ = node_->create_publisher<hexapod_msgs::msg::ControlCommand>(
      "hexapod_control/command", 10);

  leg_pose_sub_ = node_->create_subscription<hexapod_msgs::msg::LegPoses>(
      "hexapod_control/leg_pose", // topic name
      10,                         // QoS history depth
      std::bind(&HexapodControlRvizPanel::legPoseUpdateCallback, this,
                std::placeholders::_1));
}

void HexapodControlRvizPanel::legPoseUpdateCallback(
    const hexapod_msgs::msg::LegPoses msg) {

  std::string leg_name;
  geometry_msgs::msg::Point position;
  for (size_t i = 0; i < msg.leg_names.size(); i++) {
    leg_name = msg.leg_names.at(i);
    position = msg.positions.at(i);
    spinners_.at(leg_name).at(0)->blockSignals(true);
    spinners_.at(leg_name).at(1)->blockSignals(true);
    spinners_.at(leg_name).at(2)->blockSignals(true);

    spinners_.at(leg_name).at(0)->setValue(position.x);
    spinners_.at(leg_name).at(1)->setValue(position.y);
    spinners_.at(leg_name).at(2)->setValue(position.z);

    spinners_.at(leg_name).at(0)->blockSignals(false);
    spinners_.at(leg_name).at(1)->blockSignals(false);
    spinners_.at(leg_name).at(2)->blockSignals(false);
  }
}

void HexapodControlRvizPanel::onSpinnerBoxUpdate(double value) {
  QDoubleSpinBox *spinbox = qobject_cast<QDoubleSpinBox *>(sender());
  if (!spinbox)
    return;

  std::string leg_name = spinbox->property("legName").toString().toStdString();
  int axis = spinbox->property("axis").toInt();

  std::string axis_name = (axis == 0) ? "X" : (axis == 1) ? "Y" : "Z";
  RCLCPP_INFO(node_->get_logger(), "Updating value for leg=%s, %s=%.4f",
              leg_name.c_str(), axis_name.c_str(), value);

  hexapod_msgs::msg::LegPose msg;
  // Optional: Update model, emit ROS msg, etc.
  // for (int i = 0; i < 6; ++i) {
  msg.leg_name = leg_name;

  msg.position.x = spinners_.at(leg_name).at(0)->value();
  msg.position.y = spinners_.at(leg_name).at(1)->value();
  msg.position.z = spinners_.at(leg_name).at(2)->value();
  leg_pose_pub_->publish(msg);
}

void HexapodControlRvizPanel::publishCurrentPose() {
  if (current_pose_index_ >= 0 &&
      current_pose_index_ < static_cast<int>(poses_.size())) {

    const HexapodPose &pose = poses_[current_pose_index_];
    RCLCPP_DEBUG(node_->get_logger(), "Pose=%.2f,%.2f,%.2f", pose.legs[0].x,
                 pose.legs[0].y, pose.legs[0].z);
  }
}

void HexapodControlRvizPanel::updatePoseList() {
  pose_list_->clear();
  for (const auto &pose : poses_) {
    pose_list_->addItem(QString::fromStdString(pose.name));
  }

  if (!poses_.empty()) {
    pose_list_->setCurrentRow(current_pose_index_);
  }
}

void HexapodControlRvizPanel::onSavePoseClicked() {
  hexapod_msgs::msg::ControlCommand msg;
  msg.command_type = "save";
  command_pub_->publish(msg);
}

void HexapodControlRvizPanel::onDeletePoseClicked() {
  if (current_pose_index_ >= 0 &&
      current_pose_index_ < static_cast<int>(poses_.size())) {
    poses_.erase(poses_.begin() + current_pose_index_);
    if (poses_.empty()) {
      current_pose_index_ = -1;
    } else if (current_pose_index_ >= static_cast<int>(poses_.size())) {
      current_pose_index_ = poses_.size() - 1;
    }
    updatePoseList();
  }
}

void HexapodControlRvizPanel::onPoseSelected(QListWidgetItem *item) {
  current_pose_index_ = pose_list_->row(item);
  publishCurrentPose();
}

void HexapodControlRvizPanel::onPreviousPoseClicked() {
  if (!poses_.empty()) {
    current_pose_index_ =
        (current_pose_index_ - 1 + poses_.size()) % poses_.size();
    pose_list_->setCurrentRow(current_pose_index_);
    publishCurrentPose();
  }
}

void HexapodControlRvizPanel::onNextPoseClicked() {
  if (!poses_.empty()) {
    current_pose_index_ = (current_pose_index_ + 1) % poses_.size();
    pose_list_->setCurrentRow(current_pose_index_);
    publishCurrentPose();
  }
}

void HexapodControlRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);
  // TODO: Save poses to config
}

void HexapodControlRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);
  // TODO: Load poses from config
}

} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::HexapodControlRvizPanel,
                       rviz_common::Panel)
