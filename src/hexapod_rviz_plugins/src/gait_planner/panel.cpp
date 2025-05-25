#include "hexapod_rviz_plugins/gait_planner.hpp"
#include <QApplication>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <hexapod_msgs/srv/command.hpp>

#include <exception>
#include <hexapod_msgs/msg/gait.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <string>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {

GaitPlannerRvizPanel::GaitPlannerRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  leg_names_ = QStringList{"top_left",  "mid_left",  "bottom_left",
                           "top_right", "mid_right", "bottom_right"};
  setupUi();
}

GaitPlannerRvizPanel::~GaitPlannerRvizPanel() {}

void GaitPlannerRvizPanel::onInitialize() {
  DisplayContext *context = getDisplayContext();
  auto ros_node_abstraction_weak = context->getRosNodeAbstraction();
  auto ros_node_abstraction = ros_node_abstraction_weak.lock();
  if (!context) {
    throw std::runtime_error("RViz context not available");
  }
  node_ = ros_node_abstraction->get_raw_node();
  setupROS();
}

void GaitPlannerRvizPanel::setupUi() {
  QVBoxLayout *main_layout = new QVBoxLayout(this);

  // === Top Button Layout ===
  QHBoxLayout *top_buttons = new QHBoxLayout();
  QPushButton *btn_up = new QPushButton("⬆");
  QPushButton *btn_down = new QPushButton("⬇");
  QPushButton *btn_add = new QPushButton("+");
  QPushButton *btn_delete = new QPushButton("🗑");

  top_buttons->addWidget(btn_up);
  top_buttons->addWidget(btn_down);
  top_buttons->addWidget(btn_add);
  top_buttons->addWidget(btn_delete);

  // === Pose List ===
  pose_list_widget_ = new QListWidget();

  // === Bottom Button Layout ===
  QHBoxLayout *bottom_buttons = new QHBoxLayout();
  QPushButton *btn_export = new QPushButton("Export");
  QPushButton *btn_load = new QPushButton("Load");

  bottom_buttons->addWidget(btn_export);
  bottom_buttons->addWidget(btn_load);

  // === Assemble Layout ===
  main_layout->addLayout(top_buttons);
  main_layout->addWidget(pose_list_widget_);
  main_layout->addLayout(bottom_buttons);

  setLayout(main_layout);

  // === Connect Signals ===
  connect(pose_list_widget_, &QListWidget::itemClicked, this,
          &GaitPlannerRvizPanel::onPoseSelected);
  connect(btn_add, &QPushButton::clicked, this,
          &GaitPlannerRvizPanel::onAddPose);
  connect(btn_delete, &QPushButton::clicked, this,
          &GaitPlannerRvizPanel::onDeletePose);
  connect(btn_up, &QPushButton::clicked, this,
          &GaitPlannerRvizPanel::onMovePoseUp);
  connect(btn_down, &QPushButton::clicked, this,
          &GaitPlannerRvizPanel::onMovePoseDown);
  connect(pose_list_widget_, &QListWidget::itemDoubleClicked, this,
          &GaitPlannerRvizPanel::onRenamePose);
}

void GaitPlannerRvizPanel::setupROS() {
  client_ = node_->create_client<hexapod_msgs::srv::Command>("command");
}

void GaitPlannerRvizPanel::onPoseSelected(QListWidgetItem *item) {
  RCLCPP_INFO(node_->get_logger(), "Pose selected: %s",
              item->text().toStdString().c_str());
  auto command = std::make_shared<hexapod_msgs::srv::Command::Request>();
  std::string name = item->text().toStdString();
  command->pose_name = name;
  command->type = "set_pose";

  using ServiceResponseFuture =
      rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture;
  client_->async_send_request(
      command, [this, name](ServiceResponseFuture future) {
        try {
          auto response = future.get();
          RCLCPP_INFO(node_->get_logger(), "Got response: %s",
                      response->message.c_str());
        } catch (const std::exception &e) {
          RCLCPP_INFO(node_->get_logger(), "Service call failed: %s", e.what());
        }
      });
}

void GaitPlannerRvizPanel::onAddPose() {

  int count = pose_list_widget_->count();
  QString name = QString("Pose %1").arg(count + 1);

  auto command = std::make_shared<hexapod_msgs::srv::Command::Request>();
  command->pose_name = name.toStdString().c_str();
  command->type = "add_pose";

  using ServiceResponseFuture =
      rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture;
  client_->async_send_request(
      command, [this, name](ServiceResponseFuture future) {
        try {
          auto response = future.get();
          pose_list_widget_->addItem(name);
          RCLCPP_INFO(node_->get_logger(), "Got response: %s",
                      response->message.c_str());
        } catch (const std::exception &e) {
          RCLCPP_INFO(node_->get_logger(), "Service call failed: %s", e.what());
        }
      });
}

void GaitPlannerRvizPanel::onDeletePose() {
  QListWidgetItem *item = pose_list_widget_->currentItem();
  if (item) {
    delete item;
  }
}

void GaitPlannerRvizPanel::onMovePoseUp() {
  int row = pose_list_widget_->currentRow();
  if (row > 0) {
    QListWidgetItem *item = pose_list_widget_->takeItem(row);
    pose_list_widget_->insertItem(row - 1, item);
    pose_list_widget_->setCurrentItem(item);
  }
}

void GaitPlannerRvizPanel::onMovePoseDown() {
  int row = pose_list_widget_->currentRow();
  if (row < pose_list_widget_->count() - 1) {
    QListWidgetItem *item = pose_list_widget_->takeItem(row);
    pose_list_widget_->insertItem(row + 1, item);
    pose_list_widget_->setCurrentItem(item);
  }
}

void GaitPlannerRvizPanel::onRenamePose(QListWidgetItem *item) {
  bool ok;
  QString new_name = QInputDialog::getText(this, "Rename Pose",
                                           "Enter new name:", QLineEdit::Normal,
                                           item->text(), &ok);
  if (ok && !new_name.isEmpty()) {
    item->setText(new_name);
  }
}

} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::GaitPlannerRvizPanel,
                       rviz_common::Panel)
