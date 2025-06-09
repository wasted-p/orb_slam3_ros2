// TODO: Rename GaitPlanner -> ActionEditor
// TODO: Edit multiple actinos in same file
#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/gait.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/control_markers.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "hexapod_rviz_plugins/action_planner.hpp"
#include "ui/pose_list.hpp"
#include <QApplication>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
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
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <yaml-cpp/yaml.h>

#include <string>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {

// Gait planner
ActionPlannerRvizPanel::ActionPlannerRvizPanel(QWidget *parent)

    : rviz_common::Panel(parent) {
  leg_names_ = QStringList{"top_left",  "mid_left",  "bottom_left",
                           "top_right", "mid_right", "bottom_right"};
  setupUi();
}

ActionPlannerRvizPanel::~ActionPlannerRvizPanel() {}

void ActionPlannerRvizPanel::onInitialize() {
  DisplayContext *context = getDisplayContext();
  auto ros_node_abstraction_weak = context->getRosNodeAbstraction();
  auto ros_node_abstraction = ros_node_abstraction_weak.lock();
  if (!context) {
    throw std::runtime_error("RViz context not available");
  }
  node_ = ros_node_abstraction->get_raw_node();
  setupROS();
}

void ActionPlannerRvizPanel::onPoseUpdate(hexapod_msgs::msg::Pose pose) {
  // FIXME: uncomment this and add publisher
  if (current_pose < 0 || current_pose >= action_.poses.size()) {
    return;
  }
  std::map<std::string, geometry_msgs::msg::Point> buffer;

  for (size_t i = 0; i < action_.poses[current_pose].names.size(); i++) {
    hexapod_msgs::msg::Pose &pose = action_.poses[current_pose];
    buffer[pose.names[i]] = pose.positions[i];
  }

  for (size_t i = 0; i < pose.names.size(); i++)
    buffer[pose.names[i]] = pose.positions[i];

  action_.poses[current_pose].names = {};
  action_.poses[current_pose].positions = {};
  for (auto &entry : buffer) {
    action_.poses[current_pose].names.push_back(entry.first);
    action_.poses[current_pose].positions.push_back(entry.second);
  }

  auto request = std::make_shared<hexapod_msgs::srv::ControlMarkers::Request>();
  request->command = "update";
  request->poses = action_.poses;
  client_->async_send_request(
      request,
      [this](rclcpp::Client<hexapod_msgs::srv::ControlMarkers>::SharedFuture
                 future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_DEBUG(node_->get_logger(), "Updated markers successful: %s",
                       response->message.c_str());
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Updated markers failed: %s",
                       response->message.c_str());
        }
      });
}

void ActionPlannerRvizPanel::setupUi() {
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
          &ActionPlannerRvizPanel::onAddPose);
  connect(btn_delete, &QPushButton::clicked, this,
          &ActionPlannerRvizPanel::onDeletePose);
  connect(btn_up, &QPushButton::clicked, pose_list_widget_,
          &PoseList::moveCurrentPoseUp);
  connect(btn_down, &QPushButton::clicked, pose_list_widget_,
          &PoseList::moveCurrentPoseDown);

  connect(btn_export, &QPushButton::clicked, this,
          &ActionPlannerRvizPanel::onExport);
  connect(btn_load, &QPushButton::clicked, this,
          &ActionPlannerRvizPanel::onLoad);
  // Connect a signal to re-select if nothing is selected
  connect(pose_list_widget_, &PoseList::poseSelected, this,
          &ActionPlannerRvizPanel::setCurrentPose);

  connect(pose_list_widget_, &PoseList::poseMoved, this,
          &ActionPlannerRvizPanel::onPoseMoved);

  connect(pose_list_widget_, &PoseList::loopCreated, this,
          &ActionPlannerRvizPanel::createLoop);
  connect(btn_loop, &QPushButton::clicked, this,
          [this]() { pose_list_widget_->startLoopSelection(); });
  // Connect a signal to re-select if nothing is selected
}

void ActionPlannerRvizPanel::createLoop(const int from_idx, const int to_idx) {
  RCLCPP_INFO(node_->get_logger(), "%d to %d", from_idx, to_idx);
  // pose_list_widget_->addPose();
}

void ActionPlannerRvizPanel::onPoseMoved(const int from_idx, const int to_idx) {
  RCLCPP_INFO(node_->get_logger(), "Moved Poses from: %i, to:%i", from_idx,
              to_idx);
  hexapod_msgs::msg::Pose tmp = action_.poses[to_idx];
  action_.poses[to_idx] = action_.poses[from_idx];
  action_.poses[from_idx] = tmp;
};

void ActionPlannerRvizPanel::onLoad() {
  QString file_path = QFileDialog::getOpenFileName(
      this, tr("Load Gait YAML File"), QDir::currentPath(),
      tr("YAML Files (*.yaml *.yml)"));

  if (file_path.isEmpty()) {
    return; // User canceled
  }

  try {
    RCLCPP_INFO(node_->get_logger(), "File Path: %s",
                file_path.toStdString().c_str());
    YAML::Node root = YAML::LoadFile(file_path.toStdString().c_str());
    auto action_node = root["gait"];
    if (!action_node)
      throw std::runtime_error("No 'gait' key in YAML.");

    action_.name = action_node["name"].as<std::string>();
    action_.poses = {};

    auto poses_node = action_node["poses"];
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
      action_.poses.push_back(pose_msg);
    }
    pose_list_widget_->clear();
    for (hexapod_msgs::msg::Pose &pose : action_.poses) {
      pose_list_widget_->addPose(pose.name);
    }

    auto request =
        std::make_shared<hexapod_msgs::srv::ControlMarkers::Request>();
    request->command = "clear";

    auto future = client_->async_send_request(
        request,
        [this](rclcpp::Client<hexapod_msgs::srv::ControlMarkers>::SharedFuture
                   future_response) {
          auto response = future_response.get();
          if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "Cleared markers successful: %s ",
                        response->message.c_str());
          } else {
            RCLCPP_ERROR(node_->get_logger(), "Cleared markers failed: %s",
                         response->message.c_str());
          }
        });

    request->command = "add";
    request->poses = action_.poses;
    future = client_->async_send_request(
        request,
        [this](rclcpp::Client<hexapod_msgs::srv::ControlMarkers>::SharedFuture
                   future_response) {
          auto response = future_response.get();
          if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "Cleared markers successful: %s",
                        response->message.c_str());
          } else {
            RCLCPP_ERROR(node_->get_logger(), "Cleared markers failed: %s",
                         response->message.c_str());
          }
        });
    pose_pub_->publish(action_.poses[0]);
    RCLCPP_INFO(node_->get_logger(), "Loaded gait to %s",
                file_path.toStdString().c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Error Loading Gait: %s",
                 file_path.toStdString().c_str());
  }
}

void ActionPlannerRvizPanel::onExport() {
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
  file << "  name: " << action_.name << "\n";
  file << "  poses:\n";

  for (const auto &pose : action_.poses) {
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
void ActionPlannerRvizPanel::setupROS() {
  client_ =
      node_->create_client<hexapod_msgs::srv::ControlMarkers>("action/markers");

  get_pose_client_ =
      node_->create_client<hexapod_msgs::srv::GetPose>("control/pose");

  std::string POSE_TOPIC = "/hexapod/pose";
  pose_pub_ = node_->create_publisher<hexapod_msgs::msg::Pose>(POSE_TOPIC,

                                                               rclcpp::QoS(10));

  pose_sub_ = node_->create_subscription<hexapod_msgs::msg::Pose>(
      POSE_TOPIC,
      10, // QoS history depth
      std::bind(&ActionPlannerRvizPanel::onPoseUpdate, this,
                std::placeholders::_1));
}

void ActionPlannerRvizPanel::setCurrentPose(const size_t idx) {
  RCLCPP_INFO(node_->get_logger(), "Pose (%lu)", idx);

  RCLCPP_INFO(node_->get_logger(),
              "Received Set Pose Request, setting to Pose %d ", idx);
  pose_pub_->publish(action_.poses[idx]);

  RCLCPP_INFO(node_->get_logger(), "Pose set successfully");
}

void ActionPlannerRvizPanel::onAddPose() {
  RCLCPP_INFO(node_->get_logger(), "Received Add Pose Request");
  // auto new_pose = std::make_shared<hexapod_msgs::msg::Pose>();

  auto get_pose_request =
      std::make_shared<hexapod_msgs::srv::GetPose::Request>();

  get_pose_client_->async_send_request(
      get_pose_request,
      [this](rclcpp::Client<hexapod_msgs::srv::GetPose>::SharedFuture
                 future_response) {
        auto response = future_response.get();
        hexapod_msgs::msg::Pose pose = response->pose;
        pose.name = "Pose " + std::to_string(created_poses_count_ + 1);
        RCLCPP_INFO(node_->get_logger(), "Number of poses: %lu ",
                    pose.names.size());

        auto request =
            std::make_shared<hexapod_msgs::srv::ControlMarkers::Request>();
        request->command = "add";

        action_.poses.push_back(pose);
        created_poses_count_++;
        request->poses = action_.poses;

        client_->async_send_request(
            request,
            [this](
                rclcpp::Client<hexapod_msgs::srv::ControlMarkers>::SharedFuture
                    future_response) {
              auto response = future_response.get();
              if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "%s",
                            response->message.c_str());
              } else {
                RCLCPP_ERROR(node_->get_logger(), "Adding markers failed: %s",
                             response->message.c_str());
              }
            });

        current_pose = action_.poses.size() - 1;
        pose_list_widget_->addPose("Pose");
        RCLCPP_INFO(node_->get_logger(), "Added Pose %s", "Pose");
        RCLCPP_INFO(node_->get_logger(), "Current Gait");
        for (hexapod_msgs::msg::Pose &pose : action_.poses) {
          RCLCPP_INFO(node_->get_logger(), "Pose: %s", pose.name.c_str());
        }
      });
}

void ActionPlannerRvizPanel::onDeletePose() {
  size_t idx = pose_list_widget_->currentRow();

  if (idx < 0 || idx >= action_.poses.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Pose index (%d) is out of range [0, %zu)", idx,
                 action_.poses.size());
    return;
  }

  pose_list_widget_->removePose(idx);
  action_.poses.erase(action_.poses.cbegin() + idx);
  current_pose = action_.poses.size() - 1;
  if (current_pose >= 0 && current_pose <= action_.poses.size()) {

    pose_pub_->publish(action_.poses[action_.poses.size() - 1]);
    RCLCPP_INFO(node_->get_logger(), "Deleting Pose (%lu) from Gait %s", idx,
                action_.name.c_str());
  }

  auto request = std::make_shared<hexapod_msgs::srv::ControlMarkers::Request>();

  request->command = "delete";
  request->idx = idx;

  auto future = client_->async_send_request(
      request,
      [this](rclcpp::Client<hexapod_msgs::srv::ControlMarkers>::SharedFuture
                 future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node_->get_logger(), "Deleted markers successful: %s",
                      response->message.c_str());
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Deleted markers failed: %s",
                       response->message.c_str());
        }
      });

  RCLCPP_INFO(node_->get_logger(), "Current Gait");
  for (hexapod_msgs::msg::Pose &pose : action_.poses) {
    RCLCPP_INFO(node_->get_logger(), "Pose: %s", pose.name.c_str());
  }
}

//   } else if (request->type.compare("save_gait") == 0) {
//     RCLCPP_INFO(get_logger(), "Deleting Pose (%d) from Gait %s",
//                 request->pose_idx, action_.name.c_str());
//
//     action_.poses.erase(action_.poses.cbegin() + request->pose_idx);
//     current_pose = request->pose_idx - 1;
//     response->success = true;
//     response->message = "Pose Deleted successfully";
//     clearMarkers();
//     if (action_.poses.empty()) {
//       return;
//     }
//     addMarkers(action_);
//     pose_pub_->publish(action_.poses[action_.poses.size() - 1]);
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
} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::ActionPlannerRvizPanel,
                       rviz_common::Panel)
