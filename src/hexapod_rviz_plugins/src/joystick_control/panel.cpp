#include "hexapod_msgs/msg/motion.hpp"
#include <QCheckBox>
#include <QMouseEvent>
#include <QPainter>
#include <QStackedLayout>
#include <csignal>
#include <cstdlib>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_rviz_panels/joystick_control/rviz_panel.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <qboxlayout.h>
#include <qcoreapplication.h>
#include <qdir.h>
#include <qglobal.h>
#include <qwidget.h>
#include <rcl/node.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/visibility_control.hpp>
#include <rviz_common/display_context.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <unistd.h>

namespace hexapod_rviz_plugins {
JoystickRvizPanel::JoystickRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  setupUi();
  setPublishing(false);

  connect(controller, &ControllerWidget::controllerStateChanged, this,
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
  rviz_common::DisplayContext *context = getDisplayContext();
  auto ros_node_abstraction_weak = context->getRosNodeAbstraction();
  auto ros_node_abstraction = ros_node_abstraction_weak.lock();
  if (!context) {
    throw std::runtime_error("RViz context not available");
  }
  node_ = ros_node_abstraction->get_raw_node();

  setupROS();
}

void JoystickRvizPanel::setupUi() {
  main_layout_ = new QVBoxLayout;
  setLayout(main_layout_);

  // Create a vertical layout for controller
  QWidget *top_widget = new QWidget;
  int toolbar_height = 40;

  top_widget->setFixedHeight(toolbar_height);
  QVBoxLayout *top_layout = new QVBoxLayout(top_widget);

  // Create the checkbox
  QCheckBox *mode_checkbox = new QCheckBox("Publishing Mode");

  controller = new ControllerWidget;
  controller->setGeometry(0, 0, 240, 160);
  controller->setInteractionEnabled(false);

  // Add the checkbox to the layout
  top_layout->addWidget(mode_checkbox);

  main_layout_->addWidget(top_widget);

  main_layout_->addWidget(controller);
}

void JoystickRvizPanel::setupROS() {
  // Create publisher for joystick data
  joy_pub_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
  motion_pub_ =
      node_->create_publisher<hexapod_msgs::msg::Motion>("/hexapod/action", 10);

  // Create subscription for joystick data
  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      JOY_TOPIC, 10,
      std::bind(&JoystickRvizPanel::joyCallback, this, std::placeholders::_1));
}

void JoystickRvizPanel::joyCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {

  RCLCPP_DEBUG(node_->get_logger(),
               "Controller = [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
               msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3],
               msg->axes[4], msg->axes[5], msg->axes[6], msg->axes[7]);
  // controller->setControllerState(msg->axes);
  if (publisher_mode_) { // Only update in Subscriber Mode
  } else {
    // Invert inverted x axis on both left and right joystick
    msg->axes[0] = -msg->axes[0];
    msg->axes[3] = -msg->axes[3];
    controller->setValue(msg->axes);
  }
  hexapod_msgs::msg::Motion action_msg;
  const float default_axes[8] = {0, 0, 1, 0, 0, 1, 0, 0};

  bool idle = std::equal(msg->axes.begin(), msg->axes.end(), default_axes);
  if (idle) {
    action_msg.name = "rest";
  } else {
    action_msg.name = "walk";
    action_msg.type = "tripod";
    action_msg.direction = 0;
    action_msg.stride = 0;
  }
  motion_pub_->publish(action_msg);
}

void JoystickRvizPanel::setPublishing(bool state) { publisher_mode_ = state; }

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
