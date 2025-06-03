#include "hexapod_rviz_plugins/gait_planner.hpp"
#include "joystick_control/joystick.hpp"
#include "joystick_control/rviz_panel.hpp"
#include <QCheckBox>
#include <QMouseEvent>
#include <QPainter>
#include <QStackedLayout>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <qboxlayout.h>
#include <qcoreapplication.h>
#include <qdir.h>
#include <qglobal.h>
#include <qwidget.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/visibility_control.hpp>
#include <rviz_common/display_context.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <unistd.h>

namespace hexapod_rviz_plugins {
JoystickRvizPanel::JoystickRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  setupUi();
  setPublishing(false);

  connect(controller_widget, &ControllerWidget::controllerStateChanged, this,
          &JoystickRvizPanel::publishJoystickState);
}

JoystickRvizPanel::~JoystickRvizPanel() {
  if (update_timer_) {
    update_timer_->stop();
  }
  // Terminate joy node process
  if (joy_node_pid_ > 0) {
    kill(joy_node_pid_, SIGTERM);
  }
}

void JoystickRvizPanel::onInitialize() {
  setupROS();
  launchJoyNode();
}

void JoystickRvizPanel::setupUi() {
  main_layout_ = new QVBoxLayout;
  setLayout(main_layout_);

  // Create a vertical layout for controller_widget
  QWidget *top_widget = new QWidget;
  int toolbar_height = 40;

  top_widget->setFixedHeight(toolbar_height);
  QVBoxLayout *top_layout = new QVBoxLayout(top_widget);

  // Create the checkbox
  QCheckBox *mode_checkbox = new QCheckBox("Publishing Mode");

  controller_widget = new ControllerWidget;
  controller_widget->setGeometry(0, 0, 240, 160);
  controller_widget->setInteractionEnabled(false);

  // Add the checkbox to the layout
  top_layout->addWidget(mode_checkbox);

  main_layout_->addWidget(top_widget);

  main_layout_->addWidget(controller_widget);
}

void JoystickRvizPanel::setupROS() {
  rviz_common::DisplayContext *context = getDisplayContext();
  auto ros_node_abstraction_weak = context->getRosNodeAbstraction();
  auto ros_node_abstraction = ros_node_abstraction_weak.lock();
  if (!context) {
    throw std::runtime_error("RViz context not available");
  }
  node_ = ros_node_abstraction->get_raw_node();

  // Create publisher for joystick data
  joy_pub_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

  // Create subscription for joystick data
  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&JoystickRvizPanel::joyCallback, this, std::placeholders::_1));
}

void JoystickRvizPanel::launchJoyNode() {
  // Fork and execute ros2 run joy joy_node
  joy_node_pid_ = fork();
  if (joy_node_pid_ == 0) {
    // Child process
    execlp("ros2", "ros2", "run", "joy", "joy_node", nullptr);
    // If execlp fails
    RCLCPP_ERROR(node_->get_logger(), "Failed to launch joy_node");
    exit(1);
  } else if (joy_node_pid_ < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to fork process for joy_node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Launched joy_node with PID %d",
                joy_node_pid_);
  }
}

void JoystickRvizPanel::joyCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  // controller_widget->setControllerState(msg->axes);
  if (publisher_mode_) { // Only update in Subscriber Mode
    // Update joystick axes (assuming Logitech controller with at least 4 axes)

    // joystick_left_->update(); // Trigger repaint
  }
}

void JoystickRvizPanel::setPublishing(bool state) {}

void JoystickRvizPanel::publishJoystickState(const float axes[6]) {

  RCLCPP_INFO(this->node_->get_logger(),
              "Recieved: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]", axes[0],
              axes[1], axes[2], axes[3], axes[4], axes[5], axes[6], axes[7]);
  if (publisher_mode_) { // Only publish in Publisher Mode
    auto msg = std::make_unique<sensor_msgs::msg::Joy>();
    msg->axes = {-axes[0], axes[1], -axes[2], axes[3]}; // Invert X-axes
    msg->buttons = {0, 0, 0, 0, 0, 0, 0, 0}; // Placeholder, no buttons
    msg->header.stamp = node_->get_clock()->now();
    joy_pub_->publish(std::move(msg));

    // Print joystick message
    std::stringstream ss;
    ss << "Published joy message: axes=[" << -axes[0] << ", " << axes[1] << ", "
       << -axes[2] << ", " << axes[3] << "], buttons=[0, 0, 0, 0, 0, 0, 0, 0]";
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
  }
}

void JoystickRvizPanel::updatePanel() {}

void JoystickRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);
}

void JoystickRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);
}

} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::JoystickRvizPanel,
                       rviz_common::Panel)
