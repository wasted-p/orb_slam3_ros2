#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "hexapod_rviz_plugins/control_panel.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QApplication>
#include <cstddef>
#include <hexapod_msgs/msg/command.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
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
  const QStringList leg_names_ =
      QStringList{"top_left",  "mid_left",  "bottom_left",
                  "top_right", "mid_right", "bottom_right"};

  // NOTE: Populate these values dynamically from the hexapod_pose node
  double initial_pose[6][3] = {
      {0.1287, 0.0911, 0.0043},  {0.0106, 0.1264, 0.0052},
      {-0.1114, 0.1016, 0.0043}, {0.1293, -0.0917, 0.0043},
      {0.0097, -0.1246, 0.0052}, {-0.1120, -0.1004, 0.0043},
  };

  for (int i = 0; i < 6; ++i) {
    // Leg name
    QTableWidgetItem *name_item = new QTableWidgetItem(leg_names_[i]);
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    setItem(i, 0, name_item);

    std::array<QDoubleSpinBox *, 3> spin_array;
    // X, Y, Z spin boxes
    for (int j = 0; j < 3; ++j) {
      QWidget *widget = new QWidget();
      QHBoxLayout *layout = new QHBoxLayout(widget);
      layout->setContentsMargins(0, 0, 0, 0);

      QDoubleSpinBox *spin_box = createSpinBox();
      spin_box->setValue(initial_pose[i][j]);
      layout->addWidget(spin_box);
      widget->setLayout(layout);
      spin_box->setProperty("legName", leg_names_[i]);
      spin_box->setProperty("axis", j); // 0 = X, 1 = Y, 2 = Z

      setCellWidget(i, j + 1, widget);

      // Connect value changes
      connect(spin_box, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, &PoseTable::onSpinnerBoxUpdate);
      spin_array[j] = spin_box;
    }

    spinners_[leg_names_[i].toStdString()] = spin_array;
  }
  resizeColumnsToContents();
}

void PoseTable::updateSpinners(const hexapod_msgs::msg::Pose pose) {
  std::string leg_name;
  geometry_msgs::msg::Point position;
  for (size_t i = 0; i < pose.names.size(); i++) {
    leg_name = pose.names.at(i);
    position = pose.positions.at(i);
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
  spin_box->setSingleStep(0.005);
  spin_box->setDecimals(3);
  spin_box->setButtonSymbols(QAbstractSpinBox::NoButtons);
  spin_box->setStyleSheet(
      "QDoubleSpinBox { background: transparent; border: none; }");
  return spin_box;
};

HexapodControlRvizPanel::HexapodControlRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  setupUi();
  // Connect value changes
  connect(pose_table_, &PoseTable::legPoseUpdated, this,
          &HexapodControlRvizPanel::onLegPoseUpdate);
}

HexapodControlRvizPanel::~HexapodControlRvizPanel() {}

void HexapodControlRvizPanel::onLegPoseUpdate(std::string leg_name, double x,
                                              double y, double z) {
  RCLCPP_DEBUG(node_->get_logger(), "Spinner box updated");
  hexapod_msgs::msg::Pose pose;
  geometry_msgs::msg::Point position;
  position.x = x;
  position.y = y;
  position.z = z;
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
}

void HexapodControlRvizPanel::setupUi() {
  QVBoxLayout *main_layout = new QVBoxLayout;
  // Create main horizontal layout
  QHBoxLayout *horizontal_layout = new QHBoxLayout;
  // Left side - leg position table
  pose_table_ = new PoseTable;

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

  // hexapod_msgs::msg::Pose initial_pose;
  // pose_table_->updateSpinners(initial_pose);
}

void HexapodControlRvizPanel::legPoseUpdateCallback(
    const hexapod_msgs::msg::Pose pose) {
  RCLCPP_DEBUG(node_->get_logger(), "Received Pose %s", pose.name.c_str());
  for (size_t i = 0; i < pose.names.size(); i++) {
    RCLCPP_DEBUG(node_->get_logger(), "%s=[%.4f,%.4f,%.4f]",
                 pose.names[i].c_str(), pose.positions[i].x,
                 pose.positions[i].y, pose.positions[i].z);
  }

  pose_table_->updateSpinners(pose);
}
} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::HexapodControlRvizPanel,
                       rviz_common::Panel)
