

#include <hexapod_rviz_plugins/motion_editor.hpp>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {

// Gait planner
MotionEditorRvizPanel::MotionEditorRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
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
  setSelectedMotion(selected_motion_);
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

Motion &MotionEditorRvizPanel::selectedMotion() {
  return motions_[selected_motion_];
}

void MotionEditorRvizPanel::onPoseUpdate(hexapod_msgs::msg::Pose pose) {
  if (current_pose < 0)
    return;

  setMarkerArray(node_, set_marker_array_client_, selectedMotion().poses, true);
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
  QPushButton *btn_save = new QPushButton("S");
  QPushButton *btn_delete = new QPushButton("🗑");

  top_buttons->addWidget(btn_up);
  top_buttons->addWidget(btn_down);
  top_buttons->addWidget(btn_add);
  top_buttons->addWidget(btn_save);
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
  connect(
      direction_spinner, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
      this, [this](double new_yaw) {
        rotate(selectedMotion().poses, 0, 0, new_yaw - yaw_);
        yaw_ = new_yaw;
        setPose(node_, set_pose_client_, selectedMotion().poses[current_pose]);
        setMarkerArray(node_, set_marker_array_client_, selectedMotion().poses,
                       true);
      });
  connect(btn_save, &QPushButton::clicked, this, [this]() {
    getPose(node_, get_pose_client_,
            [this](const hexapod_msgs::msg::Pose pose) {
              for (size_t i = 0; i < pose.names.size(); i++) {
                RCLCPP_INFO(rclcpp::get_logger("GETTING POSE"),
                            "%s=[%.4f,%.4f,%.4f]", pose.names[i].c_str(),
                            pose.positions[i].x, pose.positions[i].y,
                            pose.positions[i].z);
              }

              selectedPose() = pose;
            });
  });
  connect(save_button, &QPushButton::pressed,
          [this]() { saveToYaml(motions_); });
  // Connect a signal to re-select if nothing is selected
  connect(pose_list_widget_, &PoseList::poseSelected, this,
          &MotionEditorRvizPanel::setCurrentPose);

  connect(pose_list_widget_, &PoseList::poseMoved, this,
          &MotionEditorRvizPanel::onPoseMoved);

  connect(motion_combo_box_, &QComboBox::currentTextChanged, this,
          [this](const QString &name) {
            this->setSelectedMotion(name.toStdString());
          });
  // Connect a signal to re - select if nothing is selected
}

void MotionEditorRvizPanel::setSelectedMotion(std::string name) {
  selected_motion_ = name;

  pose_list_widget_->clear();
  for (hexapod_msgs::msg::Pose &pose : selectedMotion().poses) {
    RCLCPP_INFO(rclcpp::get_logger("DEBUG"), "%s", pose.name.c_str());
    pose_list_widget_->addItem(pose.name.c_str());
  }
  setCurrentPose(0);
  setMarkerArray(node_, set_marker_array_client_, selectedMotion().poses);
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
  setPose(node_, set_pose_client_, selectedPose());
}

void MotionEditorRvizPanel::onAddPose() {
  auto get_pose_request =
      std::make_shared<hexapod_msgs::srv::GetPose::Request>();

  // get_pose_client_->async_send_request(
  //     get_pose_request,
  //     [this](rclcpp::Client<hexapod_msgs::srv::GetPose>::SharedFuture
  //                future_response) {
  //       auto response = future_response.get();
  //       hexapod_msgs::msg::Pose pose = response->pose;
  //       pose.name = "Pose " + std::to_string(created_poses_count_ + 1);
  //       RCLCPP_INFO(node_->get_logger(), "Number of poses: %lu ",
  //                   pose.names.size());
  //
  //       selectedMotion().poses.push_back(pose);
  //       setMarkerArray(node_, set_marker_array_client_,
  //       selectedMotion().poses); created_poses_count_++;
  //       setCurrentPose(selectedMotion().poses.size() - 1);
  //       current_pose = selectedMotion().poses.size() - 1;
  //       pose_list_widget_->addItem(pose.name.c_str());
  //       RCLCPP_INFO(node_->get_logger(), "Added Pose %s", "Pose");
  //     });
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
    setPose(node_, set_pose_client_, selectedMotion().poses[0]);
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
