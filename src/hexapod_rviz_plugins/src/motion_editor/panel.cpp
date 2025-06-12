
#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include <cmath>
#include <filesystem>
#include <hexapod_control/requests.hpp>
#include <hexapod_control/ros_constants.hpp>
#include <hexapod_rviz_plugins/motion_editor.hpp>
#include <map>
#include <qcombobox.h>
#include <qglobal.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <vector>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {

void loadFromYaml(std::map<std::string, Motion> &motions) {
  std::string base_path = "/home/thurdparty/Code/hexapod-ros/src/"
                          "hexapod_bringup/config/";
  std::string definitions_dir = base_path + "definitions/";
  std::string main_filename = base_path + "motions.yml";

  // node_->get_parameter("motion_definitions_path", main_filename);
  if (main_filename.empty())
    throw std::runtime_error("No 'motions' parameter specified.");

  YAML::Node root = YAML::LoadFile(main_filename);

  for (const auto &motion_pair : root["motions"]) {
    Motion motion;
    std::string motion_id = motion_pair.first.as<std::string>();
    const YAML::Node &motion_node = motion_pair.second;

    motion.name = motion_node["name"].as<std::string>();
    motion.category = motion_node["category"].as<std::string>("gait");
    motion.duration = motion_node["duration"].as<std::float_t>(1.0);
    motion.type = motion_node["type"].as<std::string>("cyclic");
    motion.poses = {};

    // Load poses from separate file
    std::string pose_filename = motion_node["definition"].as<std::string>();
    YAML::Node poses_root = YAML::LoadFile(pose_filename);

    for (const auto &pose_node : poses_root["poses"]) {
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

    motions[motion_id] = motion;
  }
}

void saveToYaml(std::map<std::string, Motion> &motions) {
  std::string base_path = "/home/thurdparty/Code/hexapod-ros/src/"
                          "hexapod_bringup/config/";
  std::string definitions_dir = base_path + "definitions/";
  std::string main_filename = base_path + "motions.yml";

  if (motions.empty()) {
    throw std::invalid_argument("Motions is empty");
    return;
  }

  // Create definitions directory if it doesn't exist
  std::filesystem::create_directory(definitions_dir);

  YAML::Node root;
  YAML::Node motionsnode;

  for (const auto &motion_pair : motions) {
    const std::string &motion_id = motion_pair.first;
    const Motion &motion = motion_pair.second;

    // Create main motion entry (without poses)
    YAML::Node motion_node;

    std::string pose_filename = definitions_dir + motion_id + ".yml";
    motion_node["name"] = motion.name;
    motion_node["category"] = motion.category;
    motion_node["duration"] = motion.duration;
    motion_node["type"] = motion.type;
    motion_node["definition"] = pose_filename;
    // motion_node[""] = motion.type;
    motionsnode[motion_id] = motion_node;

    // Save poses to separate file
    YAML::Node poses_root;
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

    poses_root["poses"] = poses_node;

    // Write poses file
    std::ofstream pose_file(pose_filename);
    if (!pose_file.is_open()) {
      throw std::runtime_error("Failed to open pose file for writing: " +
                               pose_filename);
    }
    pose_file << poses_root;
    pose_file.close();
  }

  root["motions"] = motionsnode;

  // Write main motion definitions file
  std::ofstream file(main_filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file for writing: " +
                             main_filename);
  }
  file << root;
  file.close();
}
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
  loadFromYaml(motions_);
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

  // selectedPose().names = {};
  // selectedPose().positions = {};
  // for (auto &entry : buffer) {
  //   selectedPose().names.push_back(entry.first);
  //   selectedPose().positions.push_back(entry.second);
  // }

  setMarkerArray(node_, set_marker_array_client_, transformedMotion().poses,
                 true);
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

  connect(direction_spinner,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this](double yaw) {
            yaw_ = yaw;
            setPose(node_, set_pose_client_,
                    transformedMotion().poses[current_pose]);
            setMarkerArray(node_, set_marker_array_client_,
                           transformedMotion().poses, true);
          });
  connect(save_button, &QPushButton::pressed,
          [this]() { saveToYaml(motions_); });

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
  setMarkerArray(node_, set_marker_array_client_, transformedMotion().poses);
}
void MotionEditorRvizPanel::onPoseMoved(const int from_idx, const int to_idx) {
  RCLCPP_INFO(node_->get_logger(), "Moved Poses from: %i, to:%i", from_idx,
              to_idx);
  hexapod_msgs::msg::Pose tmp = selectedMotion().poses[to_idx];
  selectedMotion().poses[to_idx] = selectedMotion().poses[from_idx];
  selectedMotion().poses[from_idx] = tmp;
};

void MotionEditorRvizPanel::setupROS() {
  set_marker_array_client_ =
      node_->create_client<hexapod_msgs::srv::SetMarkerArray>(
          SET_MARKER_ARRAY_SERVICE_NAME);

  get_pose_client_ =
      node_->create_client<hexapod_msgs::srv::GetPose>(GET_POSE_SERVICE_NAME);

  set_pose_client_ =
      node_->create_client<hexapod_msgs::srv::SetPose>(SET_POSE_SERVICE_NAME);

  pose_sub_ = node_->create_subscription<hexapod_msgs::msg::Pose>(
      POSE_TOPIC,
      10, // QoS history depth
      std::bind(&MotionEditorRvizPanel::onPoseUpdate, this,
                std::placeholders::_1));
}

void MotionEditorRvizPanel::setCurrentPose(const size_t idx) {
  current_pose = idx;
  setPose(node_, set_pose_client_, transformedMotion().poses[idx], true);
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
        setMarkerArray(node_, set_marker_array_client_, selectedMotion().poses);
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
    setPose(node_, set_pose_client_, transformedMotion().poses[0]);
    RCLCPP_INFO(node_->get_logger(), "Deleting Pose (%lu) from Gait %s", idx,
                selectedMotion().name.c_str());
  }

  setMarkerArray(node_, set_marker_array_client_, {});
  RCLCPP_INFO(node_->get_logger(), "Current Gait");
  for (hexapod_msgs::msg::Pose &pose : selectedMotion().poses) {
    RCLCPP_INFO(node_->get_logger(), "Pose: %s", pose.name.c_str());
  }
}

} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::MotionEditorRvizPanel,
                       rviz_common::Panel)
