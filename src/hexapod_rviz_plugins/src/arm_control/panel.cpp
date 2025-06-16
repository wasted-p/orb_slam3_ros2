#include "sensor_msgs/msg/joint_state.hpp"
#include <array>
#include <cmath>
#include <hexapod_common/requests.hpp>
#include <hexapod_rviz_panels/arm_control/panel.hpp>
#include <map>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace hexapod_rviz_plugins {

ArmJointStateControlPanel::ArmJointStateControlPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  joint_names = {
      "arm_rotator_joint",
      "arm_retractor_joint",
      "arm_abductor_joint",
  };
}

void ArmJointStateControlPanel::onStepSizeChanged(double value) {
  step_ = value;
}

void ArmJointStateControlPanel::onResetClicked() {
  for (const auto &[joint_name, spinner] : spinners_) {
    double joint_position = 0;
    spinner->setValue(joint_position);
  }
  sendJointState();
}

ArmJointStateControlPanel::~ArmJointStateControlPanel() {}

void ArmJointStateControlPanel::jointStateCallback(
    const sensor_msgs::msg::JointState &joint_state) {
  for (size_t i = 0; i < joint_state.name.size(); i++) {
    spinners_[joint_state.name[i]];
  }
}

void ArmJointStateControlPanel::onInitialize() {
  Panel::onInitialize();
  DisplayContext *context = getDisplayContext();
  auto ros_node_abstraction_weak = context->getRosNodeAbstraction();
  auto ros_node_abstraction = ros_node_abstraction_weak.lock();
  if (!context) {
    throw std::runtime_error("RViz context not available");
  }
  node_ = ros_node_abstraction->get_raw_node();

  setupUi();
  setupROS();
}

void ArmJointStateControlPanel::setupUi() {
  QVBoxLayout *main_layout = new QVBoxLayout;
  // return;
  // === Top horizontal layout (checkbox + spinner + button) ===
  QHBoxLayout *top_controls_layout = new QHBoxLayout;

  QDoubleSpinBox *step_spinbox = new QDoubleSpinBox;
  QPushButton *reset_button = new QPushButton("Reset");
  step_spinbox->setRange(0.001, 1.0); // adjust as needed
  step_spinbox->setSingleStep(0.005);
  step_spinbox->setDecimals(3);
  step_spinbox->setValue(step_); // default step size

  top_controls_layout->addWidget(step_spinbox);
  top_controls_layout->addWidget(reset_button);
  top_controls_layout->addStretch(); // pushes widgets to the left

  main_layout->addLayout(top_controls_layout);

  // === Main horizontal layout (pose table) ===
  QHBoxLayout *horizontal_layout = new QHBoxLayout;

  std::map<std::string, std::array<double, 2>> joint_limits = {
      {"arm_rotator_joint", {-M_PI, M_PI}},
      {"arm_abductor_joint", {-M_PI_2, M_PI_2}},
      {"arm_retractor_joint", {-M_PI_2, M_PI_2}},
  };

  for (const std::string &joint_name : joint_names) {
    spinners_[joint_name] = new QDoubleSpinBox;
    spinners_[joint_name]->setRange(
        joint_limits[joint_name][0],
        joint_limits[joint_name][1]); // adjust as needed
    spinners_[joint_name]->setSingleStep(0.2);
    spinners_[joint_name]->setDecimals(3);
    connect(spinners_[joint_name],
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &ArmJointStateControlPanel::sendJointState);
    horizontal_layout->addWidget(spinners_[joint_name]);
  }
  main_layout->addLayout(horizontal_layout);

  connect(step_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &ArmJointStateControlPanel::onStepSizeChanged);
  connect(reset_button, &QPushButton::clicked, this,
          &ArmJointStateControlPanel::onResetClicked);

  main_layout->addLayout(horizontal_layout);
  setLayout(main_layout);
}

void ArmJointStateControlPanel::sendJointState() {
  sensor_msgs::msg::JointState joint_state;
  for (const auto &[joint_name, spinner] : spinners_) {
    joint_state.name.push_back(joint_name);
    double joint_position = spinner->value();
    joint_state.position.push_back(joint_position);
  }
  setJointState(node_, set_joint_state_client_, joint_state);
};

void ArmJointStateControlPanel::setupROS() {
  std::string prefix_ = "arm";

  set_joint_state_client_ =
      node_->create_client<hexapod_msgs::srv::SetJointState>(
          joinWithSlash(prefix_, SET_JOINT_STATE_SERVICE_NAME));

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joinWithSlash(prefix_, JOINT_STATE_TOPIC),
      10, // QoS history depth
      std::bind(&ArmJointStateControlPanel::jointStateCallback, this,
                std::placeholders::_1));
}
}; // namespace hexapod_rviz_plugins
//
PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::ArmJointStateControlPanel,
                       rviz_common::Panel)
