#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_rviz_plugins/control_panel.hpp"
#include <QApplication>
#include <cstddef>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <qcheckbox.h>
#include <qlist.h>
#include <qspinbox.h>
#include <qtablewidget.h>
#include <rclcpp/context.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <string>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {
PoseTable::PoseTable() : QTableWidget(ROW_COUNT, COLUMN_COUNT) {

  setHorizontalHeaderLabels({"Leg", "X", "Y", "Z"});
  // verticalHeader()->setVisible(false);

  // Set leg names
  const QStringList LEG_NAMES =
      QStringList{"top_left",  "mid_left",  "bottom_left",
                  "top_right", "mid_right", "bottom_right"};

  for (int i = 0; i < 6; ++i) {
    // Leg name
    QTableWidgetItem *name_item = new QTableWidgetItem(LEG_NAMES[i]);
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    setItem(i, 0, name_item);

    std::array<QDoubleSpinBox *, 3> spin_array;
    // X, Y, Z spin boxes
    for (int j = 0; j < 3; ++j) {
      QWidget *widget = new QWidget();
      QHBoxLayout *layout = new QHBoxLayout(widget);
      layout->setContentsMargins(0, 0, 0, 0);

      QDoubleSpinBox *spin_box = createSpinBox();
      // spin_box->setValue(initial_pose[i][j]);
      layout->addWidget(spin_box);
      widget->setLayout(layout);
      spin_box->setProperty("legName", LEG_NAMES[i]);
      spin_box->setProperty("axis", j); // 0 = X, 1 = Y, 2 = Z

      setCellWidget(i, j + 1, widget);

      // Connect value changes
      connect(spin_box, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, &PoseTable::onSpinnerBoxUpdate);
      spin_array[j] = spin_box;
    }

    spinners_[LEG_NAMES[i].toStdString()] = spin_array;
  }
  resizeColumnsToContents();
}

void PoseTable::setRowValue(std::string leg_name,
                            geometry_msgs::msg::Point position) {
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

geometry_msgs::msg::Point PoseTable::getRowValue(std::string leg_name) {
  geometry_msgs::msg::Point position;
  position.x = spinners_.at(leg_name).at(0)->value();
  position.y = spinners_.at(leg_name).at(1)->value();
  position.z = spinners_.at(leg_name).at(2)->value();
  return position;
}

void PoseTable::setStep(double step) {
  for (auto &pair : spinners_) {
    pair.second.at(0)->setSingleStep(step);
    pair.second.at(1)->setSingleStep(step);
    pair.second.at(2)->setSingleStep(step);
  }
}
void PoseTable::onSpinnerBoxUpdate() {
  QDoubleSpinBox *spinbox = qobject_cast<QDoubleSpinBox *>(sender());
  if (!spinbox)
    return;

  std::string leg_name = spinbox->property("legName").toString().toStdString();
  int axis = spinbox->property("axis").toInt();

  std::string axis_name = (axis == 0) ? "X" : (axis == 1) ? "Y" : "Z";

  double x, y, z;

  hexapod_msgs::msg::Pose pose;
  geometry_msgs::msg::Point position;
  x = spinners_.at(leg_name).at(0)->value();
  y = spinners_.at(leg_name).at(1)->value();
  z = spinners_.at(leg_name).at(2)->value();

  emit legPoseUpdated(leg_name, x, y, z);
}

QDoubleSpinBox *PoseTable::createSpinBox() {
  QDoubleSpinBox *spin_box = new QDoubleSpinBox();
  spin_box->setRange(-100.0, 100.0);
  spin_box->setSingleStep(0.001);
  spin_box->setDecimals(3);
  spin_box->setButtonSymbols(QAbstractSpinBox::NoButtons);
  spin_box->setStyleSheet(
      "QDoubleSpinBox { background: transparent; border: none; }");
  return spin_box;
};

HexapodControlRvizPanel::HexapodControlRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  setupUi();
  setRelativeMode(true);
  // Connect value changes
  connect(pose_table_, &PoseTable::legPoseUpdated, this,
          &HexapodControlRvizPanel::onLegPoseUpdate);
  connect(relative_checkbox_, &QCheckBox::toggled, this,
          &HexapodControlRvizPanel::setRelativeMode);
  connect(step_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &HexapodControlRvizPanel::onStepSizeChanged);
  connect(reset_button_, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onResetClicked);
}

void HexapodControlRvizPanel::setRelativeMode(bool checked) {
  RCLCPP_INFO(rclcpp::get_logger("HexapodControl"), "Relative mode: %s",
              checked ? "ON" : "OFF");
  relative = checked;
  for (size_t i = 0; i < 6; i++) {
    geometry_msgs::msg::Point position = pose_table_->getRowValue(LEG_NAMES[i]);
    if (relative) {
      position.x -= initial_pose_[i][0];
      position.y -= initial_pose_[i][1];
      position.z -= initial_pose_[i][2];
    } else {
      position.x += initial_pose_[i][0];
      position.y += initial_pose_[i][1];
      position.z += initial_pose_[i][2];
    }

    pose_table_->setRowValue(LEG_NAMES[i], position);
  }
}

void HexapodControlRvizPanel::onStepSizeChanged(double value) {
  RCLCPP_INFO(rclcpp::get_logger("HexapodControl"), "Step size changed: %.3f",
              value);
  pose_table_->setStep(value);
}

void HexapodControlRvizPanel::onResetClicked() {
  RCLCPP_INFO(rclcpp::get_logger("HexapodControl"), "Reset button clicked");
  hexapod_msgs::msg::Pose pose;
  for (size_t i = 0; i < 6; i++) {
    geometry_msgs::msg::Point position;
    position.x = initial_pose_[i][0];
    position.y = initial_pose_[i][1];
    position.z = initial_pose_[i][2];
    pose.names.push_back(LEG_NAMES[i]);
    pose.positions.push_back(position);
  }
  pose_pub_->publish(pose);
}

HexapodControlRvizPanel::~HexapodControlRvizPanel() {}

void HexapodControlRvizPanel::onLegPoseUpdate(std::string leg_name, double x,
                                              double y, double z) {
  RCLCPP_DEBUG(node_->get_logger(), "Spinner box updated");
  hexapod_msgs::msg::Pose pose;
  geometry_msgs::msg::Point position;
  if (relative) {
    auto it = std::find(LEG_NAMES, LEG_NAMES + 6, leg_name);
    int idx = it - LEG_NAMES;
    if (idx < 6) {
      position.x = x + initial_pose_[idx][0];
      position.y = y + initial_pose_[idx][1];
      position.z = z + initial_pose_[idx][2];
    }
  } else {
    position.x = x;
    position.y = y;
    position.z = z;
  }
  pose.positions = {position};
  pose.names = {leg_name};
  pose_pub_->publish(pose);
}

void HexapodControlRvizPanel::onInitialize() {
  Panel::onInitialize();
  DisplayContext *context = getDisplayContext();
  auto ros_node_abstraction_weak = context->getRosNodeAbstraction();
  auto ros_node_abstraction = ros_node_abstraction_weak.lock();
  if (!context) {
    throw std::runtime_error("RViz context not available");
  }
  node_ = ros_node_abstraction->get_raw_node();
  setupROS();

  hexapod_msgs::msg::Pose pose;
  for (size_t i = 0; i < 6; i++) {
    geometry_msgs::msg::Point position;
    position.x = initial_pose_[i][0];
    position.y = initial_pose_[i][1];
    position.z = initial_pose_[i][2];
    if (relative) {
      position.x -= initial_pose_[i][0];
      position.y -= initial_pose_[i][1];
      position.z -= initial_pose_[i][2];
    }
    pose_table_->setRowValue(LEG_NAMES[i], position);
  }
}

void HexapodControlRvizPanel::setupUi() {
  QVBoxLayout *main_layout = new QVBoxLayout;

  // === Top horizontal layout (checkbox + spinner + button) ===
  QHBoxLayout *top_controls_layout = new QHBoxLayout;

  relative_checkbox_ = new QCheckBox("Relative");
  step_spinbox_ = new QDoubleSpinBox;
  reset_button_ = new QPushButton("Reset");
  step_spinbox_->setRange(0.001, 1.0); // adjust as needed
  step_spinbox_->setSingleStep(0.005);
  step_spinbox_->setDecimals(3);

  relative_checkbox_->toggled(true);

  top_controls_layout->addWidget(relative_checkbox_);
  top_controls_layout->addWidget(step_spinbox_);
  top_controls_layout->addWidget(reset_button_);
  top_controls_layout->addStretch(); // pushes widgets to the left

  main_layout->addLayout(top_controls_layout);

  // === Main horizontal layout (pose table) ===
  QHBoxLayout *horizontal_layout = new QHBoxLayout;
  pose_table_ = new PoseTable;

  double step = 0.005;
  step_spinbox_->setValue(step); // default step size
  pose_table_->setStep(step);
  horizontal_layout->addWidget(pose_table_, 1); // Stretch factor 1 for table

  main_layout->addLayout(horizontal_layout);
  setLayout(main_layout);
}

void HexapodControlRvizPanel::setupROS() {
  std::string POSE_TOPIC = "/hexapod/pose";
  pose_pub_ = node_->create_publisher<hexapod_msgs::msg::Pose>(POSE_TOPIC, 10);

  pose_sub_ = node_->create_subscription<hexapod_msgs::msg::Pose>(
      POSE_TOPIC,
      10, // QoS history depth
      std::bind(&HexapodControlRvizPanel::legPoseUpdateCallback, this,
                std::placeholders::_1));
}

void HexapodControlRvizPanel::legPoseUpdateCallback(
    hexapod_msgs::msg::Pose pose) {
  RCLCPP_DEBUG(node_->get_logger(), "Received Pose %s", pose.name.c_str());
  for (size_t i = 0; i < pose.names.size(); i++) {
    RCLCPP_DEBUG(node_->get_logger(), "%s=[%.4f,%.4f,%.4f]",
                 pose.names[i].c_str(), pose.positions[i].x,
                 pose.positions[i].y, pose.positions[i].z);
  }

  for (size_t i = 0; i < pose.names.size(); i++) {
    if (relative) {
      auto it = std::find(LEG_NAMES, LEG_NAMES + 6, pose.names[i]);
      int idx = it - LEG_NAMES;
      if (idx < 6) {
        pose.positions[i].x -= initial_pose_[idx][0];
        pose.positions[i].y -= initial_pose_[idx][1];
        pose.positions[i].z -= initial_pose_[idx][2];
      }
    }

    pose_table_->setRowValue(pose.names[i], pose.positions[i]);
  }
}
} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::HexapodControlRvizPanel,
                       rviz_common::Panel)
