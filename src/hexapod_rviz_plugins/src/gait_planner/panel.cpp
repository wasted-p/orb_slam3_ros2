#include "hexapod_msgs/srv/command.hpp"
#include "hexapod_rviz_plugins/gait_planner.hpp"
#include <QApplication>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <hexapod_msgs/srv/command.hpp>

#include <QFileDialog>
#include <exception>
#include <hexapod_msgs/msg/gait.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <qlist.h>
#include <rclcpp/client.hpp>
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
  pose_list_widget_->addItem("Pose 1");
  pose_list_widget_->setCurrentRow(0);
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
  pose_list_widget_->setSelectionMode(
      QAbstractItemView::SingleSelection); // Only one item selectable
  pose_list_widget_->setSelectionBehavior(
      QAbstractItemView::SelectRows); // Optional: whole row if styled
  pose_list_widget_->setDisabled(false);

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

  connect(btn_export, &QPushButton::clicked, this,
          &GaitPlannerRvizPanel::onExport);
  connect(btn_load, &QPushButton::clicked, this, &GaitPlannerRvizPanel::onLoad);
  connect(pose_list_widget_, &QListWidget::itemDoubleClicked, this,
          &GaitPlannerRvizPanel::onRenamePose);
  // Connect a signal to re-select if nothing is selected
  connect(pose_list_widget_, &QListWidget::itemSelectionChanged, [=]() {
    if (pose_list_widget_->selectedItems().isEmpty() &&
        pose_list_widget_->count() > 0) {
      pose_list_widget_->setCurrentRow(0); // or reselect previous item
    }
  });
}

void GaitPlannerRvizPanel::onLoad() {
  QString file_path = QFileDialog::getOpenFileName(
      this, tr("Load Gait YAML File"), QDir::currentPath(),
      tr("YAML Files (*.yaml *.yml)"));

  if (file_path.isEmpty()) {
    return; // User canceled
  }

  auto request = std::make_shared<hexapod_msgs::srv::Command::Request>();
  request->type = "load_gait";
  request->filepath = file_path.toStdString();

  auto future = client_->async_send_request(
      request, [this](rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture
                          future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(rclcpp::get_logger("GaitPlanner"), "Load successful: %s",
                      response->message.c_str());
          pose_list_widget_->clear();
          for (std::string pose_name : response->pose_names) {
            pose_list_widget_->addItem(pose_name.c_str());
          }
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("GaitPlanner"), "Load failed: %s",
                       response->message.c_str());
        }
      });
}

void GaitPlannerRvizPanel::onExport() {
  // Step 1: Open file dialog for saving
  QString filename = QFileDialog::getSaveFileName(
      this, tr("Export Gait as YAML"),
      QDir::currentPath() + "/sample_gait.yaml", // Prefilled filename
      tr("YAML Files (*.yaml *.yml)")            // Filter
  );

  if (filename.isEmpty()) {
    RCLCPP_WARN(node_->get_logger(), "Export cancelled by user.");
    return;
  }

  if (!filename.endsWith(".yaml") && !filename.endsWith(".yml")) {
    filename += ".yaml";
  }

  std::string path = filename.toStdString();
  RCLCPP_INFO(node_->get_logger(), "Selected export path: %s", path.c_str());

  // Step 2: Send service request with file path
  auto request = std::make_shared<hexapod_msgs::srv::Command::Request>();
  request->type = "save_gait";
  request->filepath = path;

  using FutureResponse =
      rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture;
  client_->async_send_request(request, [this](FutureResponse future) {
    try {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(node_->get_logger(), "Gait successfully saved.");
      } else {
        RCLCPP_WARN(node_->get_logger(), "Gait save failed: %s",
                    response->message.c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Service call exception: %s", e.what());
    }
  });
}
void GaitPlannerRvizPanel::setupROS() {
  client_ = node_->create_client<hexapod_msgs::srv::Command>("command");
}

void GaitPlannerRvizPanel::onPoseSelected(QListWidgetItem *item) {
  RCLCPP_INFO(node_->get_logger(), "Pose selected: %s",
              item->text().toStdString().c_str());
  auto command = std::make_shared<hexapod_msgs::srv::Command::Request>();
  size_t idx = pose_list_widget_->currentRow();
  std::string name = item->text().toStdString();
  command->pose_idx = idx;
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

  auto command = std::make_shared<hexapod_msgs::srv::Command::Request>();
  command->type = "add_pose";

  using ServiceResponseFuture =
      rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture;
  client_->async_send_request(
      command, [this, count](ServiceResponseFuture future) {
        try {
          auto response = future.get();
          std::string name = "Pose " + std::to_string(count + 1);
          pose_list_widget_->addItem(name.c_str());
          pose_list_widget_->setCurrentRow(count);
          RCLCPP_INFO(node_->get_logger(), "Got response: %s",
                      response->message.c_str());
        } catch (const std::exception &e) {
          RCLCPP_INFO(node_->get_logger(), "Service call failed: %s", e.what());
        }
      });
}

void GaitPlannerRvizPanel::onDeletePose() {
  QListWidgetItem *item = pose_list_widget_->currentItem();
  size_t idx = pose_list_widget_->currentRow();
  std::string name = item->text().toStdString();

  auto command = std::make_shared<hexapod_msgs::srv::Command::Request>();
  command->pose_idx = idx;
  command->type = "delete_pose";

  using ServiceResponseFuture =
      rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture;
  client_->async_send_request(
      command, [this, name, idx, item](ServiceResponseFuture future) {
        try {
          auto response = future.get();
          if (item)
            delete item;
          pose_list_widget_->setCurrentRow(idx - 1);
          RCLCPP_INFO(node_->get_logger(), "Got response: %s",
                      response->message.c_str());
        } catch (const std::exception &e) {
          RCLCPP_INFO(node_->get_logger(), "Service call failed: %s", e.what());
        }
      });
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
