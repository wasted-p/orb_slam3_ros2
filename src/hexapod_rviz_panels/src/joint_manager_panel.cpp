#include <hexapod_rviz_panels/joint_manager_panel.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QMessageBox>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rviz_common/display_context.hpp>  // Include this to resolve DisplayContext
#include "rclcpp/rclcpp.hpp"

namespace hexapod_rviz_panels
{
JointManagerPanel::JointManagerPanel(QWidget* parent) : Panel(parent)
{
  // Layout Setup
  const auto layout = new QVBoxLayout(this);

  // Controller selector
  controller_selector_ = new QComboBox();
  controller_selector_->addItem("Select Controller");
  controller_selector_->addItem("JointTrajectoryController");
  layout->addWidget(controller_selector_);

  // Topic selector
  topic_selector_ = new QComboBox();
  layout->addWidget(topic_selector_);

  // Spinboxes for joint states
  QLabel* joint_label = new QLabel("Joint States");
  layout->addWidget(joint_label);
  QHBoxLayout* joint_layout = new QHBoxLayout();
  
  // Creating some joint spinboxes as an example
  for (int i = 0; i < 6; ++i) {
    QSpinBox* spinbox = new QSpinBox();
    spinbox->setRange(-180, 180);  // Example range for joint angles
    joint_spinboxes_.push_back(spinbox);
    joint_layout->addWidget(spinbox);
  }

  layout->addLayout(joint_layout);

  // Save pose button
  save_pose_button_ = new QPushButton("Save Pose");
  layout->addWidget(save_pose_button_);

  // Connect signals and slots
  QObject::connect(controller_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &JointManagerPanel::onControllerChanged);
  QObject::connect(save_pose_button_, &QPushButton::released, this, &JointManagerPanel::buttonActivated);
}

JointManagerPanel::~JointManagerPanel() = default;

void JointManagerPanel::onInitialize()
{
  // Make sure that the DisplayContext is fully available here
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  
  // Fix the issue by dereferencing the shared_ptr properly
  node = node_ptr_->get_raw_node();

  trajectory_publisher_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/legs_joint_trajectory_controller/joint_trajectory", 10);

  // Create subscriptions
  subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/joint_states", 10, std::bind(&JointManagerPanel::topicCallback, this, std::placeholders::_1));

  // Fetch valid topics for controllers using ROS 2 API
  populateTopicSelector();
}

void JointManagerPanel::topicCallback(const std_msgs::msg::String& msg)
{
  // Update UI with the received joint state
  label_->setText(QString::fromStdString(msg.data));
}

void JointManagerPanel::onControllerChanged(int index)
{
  std::cout << index << std::endl;

}

void JointManagerPanel::populateTopicSelector()
{
  // Clear previous topic list
  topic_selector_->clear();

  // Use ROS 2 API to get the list of available topics
  auto node = node_ptr_->get_raw_node();
  
  // Get topics using rclcpp::Topic
  auto topics = node->get_topic_names_and_types();
  
  // Add topics to combo box
  for (const auto& topic : topics) {
    // Iterate through the vector of types for each topic
    for (const auto& type : topic.second) {
      if (type == "std_msgs/msg/String" || type == "trajectory_msgs/msg/JointTrajectory") {
        topic_selector_->addItem(QString::fromStdString(topic.first));
        break;  // Exit the inner loop once a valid type is found
      }
    }
  }
}

void JointManagerPanel::updateJointState()
{
  // Update logic for JointTrajectoryController
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  
  // For each joint, get the value from the corresponding spinbox and populate the trajectory
  for (size_t i = 0; i < joint_spinboxes_.size(); ++i) {
    trajectory_msg.points.push_back(trajectory_msgs::msg::JointTrajectoryPoint());
    trajectory_msg.points.back().positions.push_back(joint_spinboxes_[i]->value());
  }

  trajectory_publisher_->publish(trajectory_msg);
}

void JointManagerPanel::buttonActivated()
{
  // Get the current selected controller from the combobox
  int controller_index = controller_selector_->currentIndex();
  
  // Check if the selected controller is "Joint Trajectory Controller"
  if (controller_index == 1) {  // Assuming "Joint Trajectory Controller" is at index 1
    // Create a JointTrajectory message if the selected controller is JointTrajectory
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;

    // Populate the JointTrajectory message (this is an example, you can modify it according to your needs)
    joint_trajectory_msg.header.stamp.sec = 0;
    joint_trajectory_msg.header.stamp.nanosec = 0;
    joint_trajectory_msg.joint_names = {"top_left_retract_joint"};

    // Populate points for the trajectory (this is an example, modify according to your robot's joints)
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {-1};  // Example positions
    point.time_from_start = rclcpp::Duration(1, 0);  // 1 second duration

    joint_trajectory_msg.points.push_back(point);  // Add point to trajectory

    // Publish the JointTrajectory message on the appropriate topic
    trajectory_publisher_->publish(joint_trajectory_msg);  // Assuming trajectory_publisher_ is a valid publisher

    RCLCPP_INFO(node->get_logger(), "Sending Joint Trajectory" );
  } else {
    // // If not Joint Trajectory, publish the default String message
    // auto message = std_msgs::msg::String();
    // message.data = "Pose Saved";  // You can modify this message content as needed
    // publisher_->publish(message);  // Publish the String message
  }
}

}  // namespace hexapod_rviz_panels

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hexapod_rviz_panels::JointManagerPanel, rviz_common::Panel)
