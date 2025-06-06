#include "hexapod_msgs/msg/gait.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/command.hpp"
#include "hexapod_rviz_plugins/control_panel.hpp"
#include "hexapod_rviz_plugins/gait_planner.hpp"
#include "ui/pose_list.hpp"
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
#include <qevent.h>
#include <qfiledialog.h>
#include <qlist.h>
#include <qlistwidget.h>
#include <qobject.h>
#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qvariant.h>
#include <rclcpp/client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <string>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {

// Gait planner
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
  QPushButton *btn_loop = new QPushButton("⟳");

  btn_up->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  btn_down->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  btn_add->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  btn_delete->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  btn_loop->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

  btn_up->setMaximumWidth(35);
  btn_down->setMaximumWidth(35);
  btn_add->setMaximumWidth(35);
  btn_delete->setMaximumWidth(35);
  btn_loop->setMaximumWidth(35);

  btn_up->setFixedHeight(30);

  top_buttons->setSizeConstraint(QLayout::SetMinimumSize);

  top_buttons->addWidget(btn_up);
  top_buttons->addWidget(btn_down);
  top_buttons->addWidget(btn_add);
  top_buttons->addWidget(btn_loop);
  top_buttons->addWidget(btn_delete);

  // === Pose List ===
  pose_list_widget_ = new PoseList();
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
  connect(btn_add, &QPushButton::clicked, this,
          &GaitPlannerRvizPanel::onAddPose);
  connect(btn_delete, &QPushButton::clicked, this,
          &GaitPlannerRvizPanel::onDeletePose);
  connect(btn_up, &QPushButton::clicked, pose_list_widget_,
          &PoseList::moveCurrentPoseUp);
  connect(btn_down, &QPushButton::clicked, pose_list_widget_,
          &PoseList::moveCurrentPoseDown);

  connect(btn_export, &QPushButton::clicked, this,
          &GaitPlannerRvizPanel::onExport);
  connect(btn_load, &QPushButton::clicked, this, &GaitPlannerRvizPanel::onLoad);
  // Connect a signal to re-select if nothing is selected
  connect(pose_list_widget_, &PoseList::poseSelected, this,
          &GaitPlannerRvizPanel::setCurrentPose);

  connect(pose_list_widget_, &PoseList::poseMoved, this,
          &GaitPlannerRvizPanel::onPoseMoved);

  connect(pose_list_widget_, &PoseList::loopCreated, this,
          &GaitPlannerRvizPanel::createLoop);
  connect(btn_loop, &QPushButton::clicked, this,
          [this]() { pose_list_widget_->startLoopSelection(); });
  // Connect a signal to re-select if nothing is selected
}

void GaitPlannerRvizPanel::createLoop(const int from_idx, const int to_idx) {
  RCLCPP_INFO(node_->get_logger(), "%d to %d", from_idx, to_idx);
  pose_list_widget_->addPose();
}

void GaitPlannerRvizPanel::onPoseMoved(const int from_idx, const int to_idx) {
  auto request = std::make_shared<hexapod_msgs::srv::Command::Request>();
  request->type = "move_pose";
  request->pose_idx = from_idx;
  request->pose_idx_2 = to_idx;
  auto future = client_->async_send_request(
      request, [this](rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture
                          future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node_->get_logger(), "Move successful: %s",
                      response->message.c_str());
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("GaitPlanner"), "Load failed: %s",
                       response->message.c_str());
        }
      });
};
void GaitPlannerRvizPanel::onLoad() {
  QString file_path = QFileDialog::getOpenFileName(
      this, tr("Load Gait YAML File"), QDir::currentPath(),
      tr("YAML Files (*.yaml *.yml)"));

  if (file_path.isEmpty()) {
    return; // User canceled
  }

  try {
    RCLCPP_INFO(node_->get_logger(), "File Path: %s",
                file_path.toStdString().c_str());
    YAML::Node root = YAML::LoadFile(file_path);
    auto gait_node = root["gait"];
    if (!gait_node)
      throw std::runtime_error("No 'gait' key in YAML.");

    hexapod_msgs::msg::Gait gait_msg;
    gait_msg.name = gait_node["name"].as<std::string>();
    gait_msg.poses = {};

    auto poses_node = gait_node["poses"];
    if (!poses_node || !poses_node.IsSequence()) {
      throw std::runtime_error("'poses' is missing or not a sequence");
    }

    for (const auto &pose_node : poses_node) {
      hexapod_msgs::msg::Pose pose_msg;

      pose_msg.name = pose_node["name"].as<std::string>();

      // Load names
      auto names_node = pose_node["names"];
      if (names_node && names_node.IsSequence()) {
        for (const auto &name_entry : names_node) {
          pose_msg.names.push_back(name_entry.as<std::string>());
        }
      }

      // Load positions
      auto pos_node = pose_node["positions"];
      if (pos_node && pos_node.IsSequence()) {
        for (const auto &p : pos_node) {
          geometry_msgs::msg::Point pt;
          pt.x = p["x"].as<double>();
          pt.y = p["y"].as<double>();
          pt.z = p["z"].as<double>();
          pose_msg.positions.push_back(pt);
        }
      }

      gait_msg.poses.push_back(pose_msg);
    }
    // FIXME: Add marker editing functionality to action_node as service
    //  clearMarkers();
    //  addMarkers(gait_);
    //
    pose_pub_->publish(gait_.poses[0]);
    for (hexapod_msgs::msg::Pose pose : gait_.poses) {
      response->pose_names.push_back(pose.name);
    }
    response->success = true;
    response->message = "Gait loaded successfully";
    RCLCPP_INFO(this->get_logger(), "Loaded gait to %s",
                request->filepath.c_str());
  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Error occurred";
    RCLCPP_ERROR(this->get_logger(), "Error Loading Gait: %s",
                 request->filepath.c_str());
  }
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

  std::ofstream file(path);
  if (!file.is_open())
    throw std::runtime_error("Failed to open file for writing");

  file << "gait:\n";
  file << "  name: " << gait_.name << "\n";
  file << "  poses:\n";

  for (const auto &pose : gait_.poses) {
    file << "    - name: " << pose.name << "\n";
    file << "      names:\n";
    for (const auto &leg_name : pose.names) {
      file << "        - " << leg_name << "\n";
    }
    file << "      positions:\n";
    for (const auto &pos : pose.positions) {
      file << "        - {x: " << pos.x << ", y: " << pos.y << ", z: " << pos.z
           << "}\n";
    }
  }
  file.close();
}
void GaitPlannerRvizPanel::setupROS() {
  client_ = node_->create_client<hexapod_msgs::srv::Command>("command");
  pose_list_widget_->addPose();

  std::string POSE_TOPIC = "/hexapod/pose";
  pose_pub_ = node_->create_publisher<hexapod_msgs::msg::Pose>(POSE_TOPIC,
                                                               rclcpp::QoS(10));
}

void GaitPlannerRvizPanel::setCurrentPose(const size_t idx) {
  RCLCPP_INFO(node_->get_logger(), "Pose (%lu)", idx);
  auto command = std::make_shared<hexapod_msgs::srv::Command::Request>();
  command->pose_idx = idx;
  command->type = "set_pose";

  using ServiceResponseFuture =
      rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture;
  client_->async_send_request(command, [this](ServiceResponseFuture future) {
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
  auto command = std::make_shared<hexapod_msgs::srv::Command::Request>();
  command->type = "add_pose";

  using ServiceResponseFuture =
      rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture;
  client_->async_send_request(command, [this](ServiceResponseFuture future) {
    try {
      auto response = future.get();
      pose_list_widget_->addPose();
      RCLCPP_INFO(node_->get_logger(), "Got response: %s",
                  response->message.c_str());
    } catch (const std::exception &e) {
      RCLCPP_INFO(node_->get_logger(), "Service call failed: %s", e.what());
    }
  });
}

void GaitPlannerRvizPanel::onDeletePose() {
  size_t idx = pose_list_widget_->currentRow();

  auto command = std::make_shared<hexapod_msgs::srv::Command::Request>();
  command->pose_idx = idx;
  command->type = "delete_pose";

  using ServiceResponseFuture =
      rclcpp::Client<hexapod_msgs::srv::Command>::SharedFuture;
  client_->async_send_request(
      command, [this, idx](ServiceResponseFuture future) {
        try {
          auto response = future.get();
          pose_list_widget_->removePose(idx);
          RCLCPP_INFO(node_->get_logger(), "Got response: %s",
                      response->message.c_str());
        } catch (const std::exception &e) {
          RCLCPP_INFO(node_->get_logger(), "Service call failed: %s", e.what());
        }
      });
}

// void saveGaitToYamlFile(std::string path) {
// }
//
// void loadGaitFromYamlFile(const std::string &filepath,
//                           hexapod_msgs::msg::Gait &gait_msg) {
// }
// void handleCommand(
//     const std::shared_ptr<hexapod_msgs::srv::Command::Request> request,
//     std::shared_ptr<hexapod_msgs::srv::Command::Response> response) {
//
//   // Do something with request->pose...
//   RCLCPP_INFO(get_logger(), "Recieved command %s", request->type.c_str());
//   if (request->type.compare("add_pose") == 0) {
//     RCLCPP_INFO(get_logger(), "Received Add Pose Request");
//     hexapod_msgs::msg::Pose new_pose;
//     for (auto &pair : buffer) {
//       new_pose.names.push_back(pair.first);
//       new_pose.positions.push_back(pair.second);
//     }
//     gait_.poses.push_back(new_pose);
//     response->pose_names = {new_pose.name};
//     response->success = true;
//     response->message = "Pose added successfully";
//     addMarkers(gait_);
//     current_pose = gait_.poses.size() - 1;
//   } else if (request->type.compare("set_pose") == 0) {
//     RCLCPP_INFO(get_logger(), "Received Set Pose Request, setting to Pose
//     %d",
//                 request->pose_idx);
//     current_pose = request->pose_idx;
//     pose_pub_->publish(gait_.poses[current_pose]);
//     response->success = true;
//     response->message = "Pose set successfully";
//   } else if (request->type.compare("delete_pose") == 0) {
//     RCLCPP_INFO(get_logger(), "Deleting Pose (%d) from Gait %s",
//                 request->pose_idx, gait_.name.c_str());
//
//     gait_.poses.erase(gait_.poses.cbegin() + request->pose_idx);
//     current_pose = request->pose_idx - 1;
//     response->success = true;
//     response->message = "Pose Deleted successfully";
//     clearMarkers();
//     if (gait_.poses.empty()) {
//       return;
//     }
//     addMarkers(gait_);
//     pose_pub_->publish(gait_.poses[gait_.poses.size() - 1]);
//   } else if (request->type.compare("save_gait") == 0) {
//     try {
//       saveGaitToYamlFile(request->filepath);
//       response->success = true;
//       response->message = "Gait saved successfully";
//       RCLCPP_INFO(this->get_logger(), "Saved gait to %s",
//                   request->filepath.c_str());
//     } catch (const std::exception &e) {
//
//       response->message = e.what();
//       RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s",
//                    request->filepath.c_str());
//     }
//   } else if (request->type.compare("load_gait") == 0) {
//   }
// }

} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::GaitPlannerRvizPanel,
                       rviz_common::Panel)
