#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include <QApplication>
#include <cstddef>
#include <hexapod_common/hexapod.hpp>
#include <hexapod_common/logging.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_common/yaml_utils.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_rviz_panels/hexapod_control/panel.hpp>
#include <map>
#include <pluginlib/class_list_macros.hpp>
#include <qcheckbox.h>
#include <qlist.h>
#include <qpushbutton.h>
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
geometry_msgs::msg::Point point(double x, double y, double z) {
  geometry_msgs::msg::Point position;
  position.x = x;
  position.y = y;
  position.z = z;
  return position;
} // namespace hexapod_rviz_plugins

hexapod_msgs::msg::Pose
toAbsolute(const hexapod_msgs::msg::Pose &pose,
           std::map<std::string, geometry_msgs::msg::Point> initial_pose) {
  hexapod_msgs::msg::Pose absolute_pose;
  for (size_t i = 0; i < pose.names.size(); i++) {
    geometry_msgs::msg::Point position = pose.positions[i];
    const std::string &leg_name = pose.names[i];
    position.x += initial_pose[leg_name].x;
    position.y += initial_pose[leg_name].y;
    position.z += initial_pose[leg_name].z;
    absolute_pose.names.push_back(leg_name);
    absolute_pose.positions.push_back(position);
  }
  return absolute_pose;
}

hexapod_msgs::msg::Pose
toRelative(const hexapod_msgs::msg::Pose &pose,
           std::map<std::string, geometry_msgs::msg::Point> initial_pose) {
  hexapod_msgs::msg::Pose relative_pose;
  for (size_t i = 0; i < pose.names.size(); i++) {
    geometry_msgs::msg::Point position = pose.positions[i];
    const std::string &leg_name = pose.names[i];
    position.x -= initial_pose[leg_name].x;
    position.y -= initial_pose[leg_name].y;
    position.z -= initial_pose[leg_name].z;
    relative_pose.names.push_back(leg_name);
    relative_pose.positions.push_back(position);
  }
  return relative_pose;
}

PoseTable::PoseTable() : QTableWidget(ROW_COUNT, COLUMN_COUNT) {
  setHorizontalHeaderLabels({"Leg", "X", "Y", "Z"});
  // verticalHeader()->setVisible(false);

  // Set leg names

  for (int i = 5; i >= 0; i--) {
    // Leg name
    QTableWidgetItem *name_item = new QTableWidgetItem(LEG_NAMES[i].c_str());
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
      spin_box->setProperty("legName", LEG_NAMES[i].c_str());
      spin_box->setProperty("axis", j); // 0 = X, 1 = Y, 2 = Z

      setCellWidget(i, j + 1, widget);

      // Connect value changes
      connect(spin_box, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, &PoseTable::onSpinnerBoxUpdate);
      spin_array[j] = spin_box;
    }

    spinners_[LEG_NAMES[i].c_str()] = spin_array;
  }
  resizeColumnsToContents();
}

hexapod_msgs::msg::Pose PoseTable::getPose() {
  hexapod_msgs::msg::Pose pose;
  for (const std::string &leg_name : LEG_NAMES) {
    geometry_msgs::msg::Point position = getRowValue(leg_name);
    pose.names.push_back(leg_name);
    pose.positions.push_back(position);
  }
  return pose;
};

void PoseTable::setPose(const hexapod_msgs::msg::Pose &pose) {
  for (size_t i = 0; i < pose.names.size(); i++) {
    const std::string &leg_name = pose.names[i];
    const geometry_msgs::msg::Point position = pose.positions[i];
    setRowValue(leg_name, position);
  }
};

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
  relative = true;

  initial_pose_ = {
      {"top_left", point(0.1287, 0.0911, 0.0043)},
      {"mid_left", point(0.0106, 0.1264, 0.0052)},
      {"bottom_left", point(-0.1114, 0.1016, 0.0043)},
      {"top_right", point(0.1293, -0.0917, 0.0043)},
      {"mid_right", point(0.0097, -0.1246, 0.0052)},
      {"bottom_right", point(-0.1120, -0.1004, 0.0043)},
  };

  setupUi();
  // setRelativeMode(true);
  // Connect value changes
}

void HexapodControlRvizPanel::setRelativeMode(bool relative) {
  this->relative = relative;
  hexapod_msgs::msg::Pose pose = pose_table_->getPose();
  if (relative) {
    pose = toRelative(pose, initial_pose_);
  } else {
    pose = toAbsolute(pose, initial_pose_);
  }
  pose_table_->setPose(pose);
}

void HexapodControlRvizPanel::onStepSizeChanged(double value) {
  pose_table_->setStep(value);
}

void HexapodControlRvizPanel::onResetClicked() {
  hexapod_msgs::msg::Pose pose;
  for (const auto &entry : initial_pose_) {
    geometry_msgs::msg::Point position;
    position.x = entry.second.x;
    position.y = entry.second.y;
    position.z = entry.second.z;
    pose.names.push_back(entry.first);
    pose.positions.push_back(position);
  }
  pose_table_->setPose(pose);
}

HexapodControlRvizPanel::~HexapodControlRvizPanel() {}

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
  geometry_msgs::msg::Point position;
  for (const std::string &leg_name : LEG_NAMES) {
    pose.names.push_back(leg_name);
    pose.positions.push_back(initial_pose_[leg_name]);
  }
}

void HexapodControlRvizPanel::setupUi() {
  QVBoxLayout *main_layout = new QVBoxLayout;

  // return;
  // === Top horizontal layout (checkbox + spinner + button) ===
  QHBoxLayout *top_controls_layout = new QHBoxLayout;

  QCheckBox *relative_checkbox = new QCheckBox("Relative");
  QDoubleSpinBox *step_spinbox = new QDoubleSpinBox;
  QPushButton *reset_button = new QPushButton("Reset");
  QPushButton *execute_button = new QPushButton("Execute");

  step_spinbox->setRange(0.001, 1.0); // adjust as needed
  step_spinbox->setSingleStep(0.005);
  step_spinbox->setDecimals(3);

  relative_checkbox->toggled(relative);

  top_controls_layout->addWidget(relative_checkbox);
  top_controls_layout->addWidget(step_spinbox);
  top_controls_layout->addWidget(execute_button);
  top_controls_layout->addWidget(reset_button);
  top_controls_layout->addStretch(); // pushes widgets to the left

  main_layout->addLayout(top_controls_layout);

  // === Main horizontal layout (pose table) ===
  QHBoxLayout *horizontal_layout = new QHBoxLayout;
  pose_table_ = new PoseTable;

  double step = 0.005;
  step_spinbox->setValue(step); // default step size
  pose_table_->setStep(step);
  horizontal_layout->addWidget(pose_table_, 1); // Stretch factor 1 for table

  connect(relative_checkbox, &QCheckBox::toggled, this,
          &HexapodControlRvizPanel::setRelativeMode);
  connect(step_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &HexapodControlRvizPanel::onStepSizeChanged);
  connect(execute_button, &QPushButton::clicked, this, [this]() {
    hexapod_msgs::msg::Pose pose = pose_table_->getPose();
    if (relative)
      pose = toAbsolute(pose, initial_pose_);
    RCLCPP_INFO(get_logger("TEST"), "EXECUTING POSE");
    setPose(node_->shared_from_this(), set_pose_client_, pose);
  });
  connect(reset_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onResetClicked);

  main_layout->addLayout(horizontal_layout);
  setLayout(main_layout);
}

void HexapodControlRvizPanel::setupROS() {
  std::string service_name, topic_name;
  std::string prefix_ = "hexapod";

  set_pose_client_ = node_->create_client<hexapod_msgs::srv::SetPose>(
      joinWithSlash(prefix_, SET_POSE_SERVICE_NAME));
  // std::string POSE_TOPIC = "/hexapod/pose";

  pose_sub_ = node_->create_subscription<hexapod_msgs::msg::Pose>(
      joinWithSlash(prefix_, POSE_TOPIC),
      10, // QoS history depth
      std::bind(&HexapodControlRvizPanel::legPoseUpdateCallback, this,
                std::placeholders::_1));
}

void HexapodControlRvizPanel::legPoseUpdateCallback(
    hexapod_msgs::msg::Pose pose) {
  if (relative) {
    pose = toRelative(pose, initial_pose_);
  }
  pose_table_->setPose(pose);
  // for (size_t i = 0; i < pose.names.size(); i++) {
  //   pose_table_->setRowValue(pose.names[i], pose.positions[i]);
  // }
}
} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::HexapodControlRvizPanel,
                       rviz_common::Panel)
