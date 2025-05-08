#include "hexapod_control/arm_control_tab.hpp"
#include <random>

ArmControlTab::ArmControlTab(QWidget *parent) : QWidget(parent) {
  // Initialize UI components
  joint1SpinBox_ = new QDoubleSpinBox(this);
  joint2SpinBox_ = new QDoubleSpinBox(this);
  joint3SpinBox_ = new QDoubleSpinBox(this);
  executeButton_ = new QPushButton("Execute", this);
  randomButton_ = new QPushButton("Random", this);
  resetButton_ = new QPushButton("Reset", this);

  // Configure spin boxes
  for (auto *spinBox : {joint1SpinBox_, joint2SpinBox_, joint3SpinBox_}) {
    spinBox->setRange(MIN_ANGLE, MAX_ANGLE);
    spinBox->setDecimals(4);
    spinBox->setSingleStep(0.05);
    spinBox->setAlignment(Qt::AlignRight);
    spinBox->setValue(0.0);
  }

  // Setup layout
  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(new QLabel("Arm Rotator Joint", this));
  mainLayout->addWidget(joint1SpinBox_);
  mainLayout->addWidget(new QLabel("Arm Abductor Joint", this));
  mainLayout->addWidget(joint2SpinBox_);
  mainLayout->addWidget(new QLabel("Arm Retractor Joint", this));
  mainLayout->addWidget(joint3SpinBox_);
  mainLayout->addWidget(executeButton_);
  mainLayout->addWidget(randomButton_);
  mainLayout->addWidget(resetButton_);
  mainLayout->addStretch();
  setLayout(mainLayout);

  // Initialize ROS node
  node_ = NodeManager::getNode();
  pub_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable());
  pub_joint_group_command_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/arm_joint_group_position_controller/commands",
          rclcpp::QoS(10).reliable());

  // Connect signals
  connect(executeButton_, &QPushButton::clicked, this,
          &ArmControlTab::onExecuteButtonClicked);
  connect(randomButton_, &QPushButton::clicked, this,
          &ArmControlTab::onRandomButtonClicked);
  connect(resetButton_, &QPushButton::clicked, this,
          &ArmControlTab::onResetButtonClicked);

  // Publish initial joint state
  publishJointStates();
}

void ArmControlTab::publishJointStates() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->get_clock()->now();

  double values[] = {joint1SpinBox_->value(), joint2SpinBox_->value(),
                     joint3SpinBox_->value()};
  for (size_t i = 0; i < jointNames_.size(); ++i) {
    msg.name.push_back(jointNames_[i].toStdString());
    msg.position.push_back(values[i]);
    msg.velocity.push_back(0.0);
  }

  pub_joint_states_->publish(msg);
  RCLCPP_DEBUG(node_->get_logger(),
               "Published arm joint states to /joint_states");
}

void ArmControlTab::publishJointGroupCommand() {
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {joint1SpinBox_->value(), joint2SpinBox_->value(),
              joint3SpinBox_->value()};

  pub_joint_group_command_->publish(msg);
  RCLCPP_DEBUG(node_->get_logger(), "Published arm joint command to "
                                    "/arm_joint_group_position_controller/"
                                    "joint_group_position_controller/command");

  // Reset UI state
  is_executing_ = false;
  executeButton_->setText("Execute");
  executeButton_->setEnabled(true);
}

void ArmControlTab::onExecuteButtonClicked() {
  if (is_executing_) {
    return;
  }

  is_executing_ = true;
  executeButton_->setText("Executing...");
  executeButton_->setEnabled(false);

  publishJointGroupCommand();
}

void ArmControlTab::onRandomButtonClicked() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(MIN_ANGLE, MAX_ANGLE);

  joint1SpinBox_->setValue(dis(gen));
  joint2SpinBox_->setValue(dis(gen));
  joint3SpinBox_->setValue(dis(gen));

  publishJointStates();
}

void ArmControlTab::onResetButtonClicked() {
  joint1SpinBox_->setValue(0.0);
  joint2SpinBox_->setValue(0.0);
  joint3SpinBox_->setValue(0.0);

  publishJointStates();
}
