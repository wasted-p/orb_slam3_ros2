
#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/control_markers.hpp"
#include <cmath>
#include <hexapod_rviz_plugins/motion_editor.hpp>
#include <qcombobox.h>
#include <qglobal.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <sys/types.h>
#include <vector>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {

void sendSetMarkersRequest(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<hexapod_msgs::srv::ControlMarkers>::SharedPtr client,
    const std::vector<hexapod_msgs::msg::Pose> &poses, bool update = false) {
  auto request = std::make_shared<hexapod_msgs::srv::ControlMarkers::Request>();
  if (update) {
    request->command = "update";
  } else {
    request->command = "add";
  }

  request->poses = {poses};

  client->async_send_request(
      request,
      [node](rclcpp::Client<hexapod_msgs::srv::ControlMarkers>::SharedFuture
                 future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node->get_logger(), "%s", response->message.c_str());
        } else {
          RCLCPP_ERROR(node->get_logger(), "Adding markers failed: %s",
                       response->message.c_str());
        }
      });
};
// Gait planner
MotionEditorRvizPanel::MotionEditorRvizPanel(QWidget *parent)

    : rviz_common::Panel(parent) {
  leg_names_ = QStringList{"top_left",  "mid_left",  "bottom_left",
                           "top_right", "mid_right", "bottom_right"};
  setupUi();
}

MotionEditorRvizPanel::~MotionEditorRvizPanel() {}

void MotionEditorRvizPanel::onInitialize() {
  DisplayContext *context = getDisplayContext();
  auto ros_node_abstraction_weak = context->getRosNodeAbstraction();
  auto ros_node_abstraction = ros_node_abstraction_weak.lock();
  if (!context) {
    throw std::runtime_error("RViz context not available");
  }
  node_ = ros_node_abstraction->get_raw_node();
  setupROS();
  loadMotions();
  for (auto &entry : motions_) {
    motion_combo_box_->addItem(entry.first.c_str());
  }
}

hexapod_msgs::msg::Pose &MotionEditorRvizPanel::selectedPose() {
  return selectedMotion().poses[current_pose];
}

void rotate(std::vector<hexapod_msgs::msg::Pose> &poses, double roll,
            double pitch, double yaw) {
  if (poses.empty())
    return;

  // Store first points for each leg as pivot points
  std::map<std::string, geometry_msgs::msg::Point> pivot_points;

  // First pass: collect first points for each leg
  const auto &first_pose = poses[0];
  for (size_t leg_i = 0; leg_i < first_pose.names.size(); leg_i++) {
    const auto &leg_name = first_pose.names[leg_i];
    const auto &position = first_pose.positions[leg_i];
    pivot_points[leg_name] = position;
  }

  // Create rotation transform
  tf2::Transform rotation_transform;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  rotation_transform.setRotation(q);
  rotation_transform.setOrigin(tf2::Vector3(0, 0, 0));

  // Rotate all points in all poses
  for (auto &pose : poses) {
    for (size_t leg_i = 0; leg_i < pose.names.size(); leg_i++) {
      const auto &leg_name = pose.names[leg_i];
      auto &position = pose.positions[leg_i];

      // Get pivot point for this leg
      const auto &pivot = pivot_points[leg_name];
      tf2::Vector3 pivot_vec(pivot.x, pivot.y, pivot.z);

      // Convert to tf2::Vector3
      tf2::Vector3 tf_point(position.x, position.y, position.z);

      // Translate to pivot
      tf_point -= pivot_vec;

      // Apply rotation
      tf_point = rotation_transform * tf_point;

      // Translate back
      tf_point += pivot_vec;

      // Convert back to geometry_msgs::Point
      position.x = tf_point.x();
      position.y = tf_point.y();
      position.z = tf_point.z();
    }
  }
}

void scale(std::vector<hexapod_msgs::msg::Pose> &poses, double x, double y,
           double z) {
  if (poses.empty())
    return;

  // Rotate all points in all poses
  for (auto &pose : poses) {
    for (size_t leg_i = 0; leg_i < pose.names.size(); leg_i++) {
      const auto &leg_name = pose.names[leg_i];
      auto &position = pose.positions[leg_i];
    }
  }
}

Motion &MotionEditorRvizPanel::selectedMotion() {
  return motions_[selected_motion_];
}

void MotionEditorRvizPanel::onPoseUpdate(hexapod_msgs::msg::Pose pose) {
  if (current_pose < 0)
    return;

  std::map<std::string, geometry_msgs::msg::Point> buffer;

  for (size_t i = 0; i < selectedPose().names.size(); i++) {
    buffer[selectedPose().names[i]] = selectedPose().positions[i];
  }

  for (size_t i = 0; i < pose.names.size(); i++)
    buffer[pose.names[i]] = pose.positions[i];

  selectedPose().names = {};
  selectedPose().positions = {};
  for (auto &entry : buffer) {
    selectedPose().names.push_back(entry.first);
    selectedPose().positions.push_back(entry.second);
  }

  sendSetMarkersRequest(node_, client_, transformedMotion().poses, true);
}

Motion MotionEditorRvizPanel::transformedMotion() {
  Motion transformed_motion = selectedMotion();
  rotate(transformed_motion.poses, 0, 0, yaw_);
  scale(transformed_motion.poses, effort, 1, 1);
  return transformed_motion;
}

void MotionEditorRvizPanel::setupUi() {
  QVBoxLayout *main_layout = new QVBoxLayout(this);

  motion_combo_box_ = new QComboBox;

  main_layout->addWidget(motion_combo_box_);

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

  QHBoxLayout *param_inputs_layout = new QHBoxLayout();
  QDoubleSpinBox *direction_spinner = new QDoubleSpinBox;
  direction_spinner->setSuffix(" rad");
  direction_spinner->setMaximum(M_PI);
  direction_spinner->setMinimum(0);
  direction_spinner->setSingleStep(0.1);
  param_inputs_layout->addWidget(direction_spinner);

  // === Pose List ===
  pose_list_widget_ = new PoseList();
  pose_list_widget_->setSelectionMode(
      QAbstractItemView::SingleSelection); // Only one item selectable
  pose_list_widget_->setSelectionBehavior(
      QAbstractItemView::SelectRows); // Optional: whole row if styled
  pose_list_widget_->setDisabled(false);

  // === Bottom Button Layout ===
  QHBoxLayout *bottom_buttons = new QHBoxLayout();

  QPushButton *save_button = new QPushButton("🗑");

  connect(
      direction_spinner, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
      this, [this](double yaw) {
        yaw_ = yaw;
        sendSetMarkersRequest(node_, client_, transformedMotion().poses, true);
      });
  connect(save_button, &QPushButton::pressed, [this]() { saveMotions(); });

  bottom_buttons->addWidget(save_button);
  // === Assemble Layout ===
  main_layout->addLayout(top_buttons);
  main_layout->addLayout(param_inputs_layout);
  main_layout->addWidget(pose_list_widget_);
  main_layout->addLayout(bottom_buttons);

  setLayout(main_layout);

  // === Connect Signals ===
  connect(btn_add, &QPushButton::clicked, this,
          &MotionEditorRvizPanel::onAddPose);
  connect(btn_delete, &QPushButton::clicked, this,
          &MotionEditorRvizPanel::onDeletePose);
  connect(btn_up, &QPushButton::clicked, pose_list_widget_,
          &PoseList::moveCurrentPoseUp);
  connect(btn_down, &QPushButton::clicked, pose_list_widget_,
          &PoseList::moveCurrentPoseDown);
  // Connect a signal to re-select if nothing is selected
  connect(pose_list_widget_, &PoseList::poseSelected, this,
          &MotionEditorRvizPanel::setCurrentPose);

  connect(pose_list_widget_, &PoseList::poseMoved, this,
          &MotionEditorRvizPanel::onPoseMoved);

  connect(motion_combo_box_, &QComboBox::currentTextChanged, this,
          [this](const QString &name) {
            this->setSelectedMotion(name.toStdString());
          });
  // Connect a signal to re-select if nothing is selected
}

void MotionEditorRvizPanel::setSelectedMotion(std::string name) {
  selected_motion_ = name.c_str();
  pose_list_widget_->clear();
  for (hexapod_msgs::msg::Pose &pose : selectedMotion().poses) {
    pose_list_widget_->addPose(pose.name);
  }
  selected_motion_ = name;
  setCurrentPose(0);
  sendSetMarkersRequest(node_, client_, selectedMotion().poses);
}
void MotionEditorRvizPanel::onPoseMoved(const int from_idx, const int to_idx) {
  RCLCPP_INFO(node_->get_logger(), "Moved Poses from: %i, to:%i", from_idx,
              to_idx);
  hexapod_msgs::msg::Pose tmp = selectedMotion().poses[to_idx];
  selectedMotion().poses[to_idx] = selectedMotion().poses[from_idx];
  selectedMotion().poses[from_idx] = tmp;
};

void saveAction(const std::vector<hexapod_msgs::msg::Pose> &poses,
                const std::string &filename) {
  std::ofstream file(filename, std::ios::binary);

  // Write header
  size_t num_points = poses.size();
  size_t num_joints = poses[0].names.size();

  file.write(reinterpret_cast<const char *>(&num_points), sizeof(num_points));
  file.write(reinterpret_cast<const char *>(&num_joints), sizeof(num_joints));

  // Write trajectory data
  for (const auto &pose : poses) {
    file.write(reinterpret_cast<const char *>(&pose.name), sizeof(double));
    file.write(reinterpret_cast<const char *>(pose.positions.data()),
               num_joints * sizeof(double));
  }
  file.close();
}

void MotionEditorRvizPanel::saveMotions() {
  std::string filename = "/home/thurdparty/Code/hexapod-ros/src/"
                         "hexapod_bringup/config/motion_definitions.yml";
  // node_->get_parameter("motion_definitions_path", filename);

  if (filename.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No 'motions' parameter specified.");
    return;
  }

  if (motions_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No motions to save.");
    return;
  }

  try {
    YAML::Node root;
    YAML::Node motions_node;

    RCLCPP_INFO(node_->get_logger(), "Saving motions:");

    for (const auto &motion_pair : motions_) {
      const std::string &motion_id = motion_pair.first;
      const Motion &motion = motion_pair.second;

      RCLCPP_INFO(node_->get_logger(), " - Motion : %s", motion_id.c_str());

      YAML::Node motion_node;
      motion_node["name"] = motion.name;
      motion_node["category"] = motion.category;
      motion_node["duration"] = motion.duration;
      motion_node["type"] = motion.type;

      YAML::Node poses_node;
      for (const auto &pose_msg : motion.poses) {
        YAML::Node pose_node;
        pose_node["name"] = pose_msg.name;

        YAML::Node names_node;
        for (const auto &name : pose_msg.names) {
          names_node.push_back(name);
        }
        pose_node["names"] = names_node;

        YAML::Node positions_node;
        for (const auto &position : pose_msg.positions) {
          YAML::Node pos_node;
          pos_node["x"] = position.x;
          pos_node["y"] = position.y;
          pos_node["z"] = position.z;
          positions_node.push_back(pos_node);
        }
        pose_node["positions"] = positions_node;

        poses_node.push_back(pose_node);
      }
      motion_node["poses"] = poses_node;

      motions_node[motion_id] = motion_node;
    }

    root["motions"] = motions_node;

    std::ofstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open file for writing: %s",
                   filename.c_str());
      return;
    }

    file << root;
    file.close();

    RCLCPP_INFO(node_->get_logger(), "Successfully saved %lu motions to %s",
                motions_.size(), filename.c_str());

  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "YAML error while saving motions: %s",
                 e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Error while saving motions: %s",
                 e.what());
  }
}

void MotionEditorRvizPanel::loadMotions() {

  std::string filename = "/home/thurdparty/Code/hexapod-ros/src/"
                         "hexapod_bringup/config/motion_definitions.yml";

  // node_->get_parameter("motion_definitions_path", filename);

  if (filename.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No 'motions' parameter specified.");
    return;
  }

  YAML::Node root = YAML::LoadFile(filename);

  RCLCPP_INFO(node_->get_logger(), "Motions:");

  for (const auto &motion_pair : root["motions"]) {
    Motion motion;
    std::string motion_id = motion_pair.first.as<std::string>();
    const YAML::Node &motion_node = motion_pair.second;

    RCLCPP_INFO(node_->get_logger(), " - Motion : %s", motion_id.c_str());

    motion.name = motion_node["name"].as<std::string>();
    motion.category = motion_node["category"].as<std::string>("gait");
    motion.duration = motion_node["duration"].as<std::float_t>(1.0);
    motion.type = motion_node["type"].as<std::string>("cyclic");

    motion.poses = {};

    for (const auto &pose_node : motion_node["poses"]) {
      hexapod_msgs::msg::Pose pose_msg;
      pose_msg.name = pose_node["name"].as<std::string>();

      for (const auto &n : pose_node["names"]) {
        pose_msg.names.push_back(n.as<std::string>());
      }

      for (const auto &pos : pose_node["positions"]) {
        geometry_msgs::msg::Point p;
        p.x = pos["x"].as<double>();
        p.y = pos["y"].as<double>();
        p.z = pos["z"].as<double>();
        pose_msg.positions.push_back(p);
      }

      motion.poses.push_back(pose_msg);
    }
    motions_[motion_id] = motion;
  }

  RCLCPP_INFO(node_->get_logger(), "Loaded %lu motions.", motions_.size());
}

void publishPose() {}
void MotionEditorRvizPanel::setupROS() {
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
      std::bind(&MotionEditorRvizPanel::onPoseUpdate, this,
                std::placeholders::_1));
}

void MotionEditorRvizPanel::setCurrentPose(const size_t idx) {
  current_pose = idx;
  pose_pub_->publish(transformedMotion().poses[idx]);
  RCLCPP_INFO(node_->get_logger(), "Pose (%lu) Selected Successfully", idx);
}

void MotionEditorRvizPanel::onAddPose() {
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

        selectedMotion().poses.push_back(pose);
        sendSetMarkersRequest(node_, client_, selectedMotion().poses);
        created_poses_count_++;
        setCurrentPose(selectedMotion().poses.size() - 1);
        current_pose = selectedMotion().poses.size() - 1;
        pose_list_widget_->addPose(pose.name);
        RCLCPP_INFO(node_->get_logger(), "Added Pose %s", "Pose");
      });
}

void MotionEditorRvizPanel::onDeletePose() {
  size_t idx = pose_list_widget_->currentRow();

  if (idx < 0 || idx >= selectedMotion().poses.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Pose index (%lu) is out of range [0, %zu)", idx,
                 selectedMotion().poses.size());
    return;
  }

  pose_list_widget_->removePose(idx);
  selectedMotion().poses.erase(selectedMotion().poses.cbegin() + idx);
  current_pose = selectedMotion().poses.size() - 1;
  if (current_pose >= 0 && current_pose <= selectedMotion().poses.size()) {

    pose_pub_->publish(
        selectedMotion().poses[selectedMotion().poses.size() - 1]);
    RCLCPP_INFO(node_->get_logger(), "Deleting Pose (%lu) from Gait %s", idx,
                selectedMotion().name.c_str());
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
  for (hexapod_msgs::msg::Pose &pose : selectedMotion().poses) {
    RCLCPP_INFO(node_->get_logger(), "Pose: %s", pose.name.c_str());
  }
}

} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::MotionEditorRvizPanel,
                       rviz_common::Panel)
