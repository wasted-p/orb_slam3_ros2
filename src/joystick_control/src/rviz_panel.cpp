#include "joystick_control/rviz_panel.hpp"
#include <QCheckBox>
#include <QMouseEvent>
#include <QPainter>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <unistd.h>

namespace joystick_rviz_panel {

JoystickWidget::JoystickWidget(QWidget *parent) : QWidget(parent) {
  setMinimumSize(200, 100);
  setMouseTracking(true);
}

void JoystickWidget::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw left joystick (outer circle)
  painter.setBrush(Qt::gray);
  painter.drawEllipse(30, 20, 60, 60);

  // Draw left stick (inner circle, axes 0, 1)
  float left_x = -axes_values_[0] * 20; // Invert X-axis
  float left_y = axes_values_[1] * 20;
  painter.setBrush(Qt::darkGray);
  painter.drawEllipse(QPointF(60 + left_x, 50 - left_y), 10, 10);

  // Draw right joystick (outer circle)
  painter.setBrush(Qt::gray);
  painter.drawEllipse(110, 20, 60, 60);

  // Draw right stick (inner circle, axes 2, 3)
  float right_x = -axes_values_[3] * 20; // Invert X-axis, horizontal movement
  float right_y = axes_values_[4] * 20;  // Y-axis, vertical movement
  painter.setBrush(Qt::darkGray);
  painter.drawEllipse(QPointF(140 + right_x, 50 - right_y), 10, 10);
}

void JoystickWidget::mousePressEvent(QMouseEvent *event) {
  if (!is_interactive_)
    return; // Disable dragging in Subscriber Mode
  QPointF pos = event->pos();
  // Check if click is within left joystick
  if (QPointF(pos - QPointF(60, 50)).manhattanLength() < 30) {
    dragging_left_ = true;
  }
  // Check if click is within right joystick
  else if (QPointF(pos - QPointF(140, 50)).manhattanLength() < 30) {
    dragging_right_ = true;
  }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent *event) {
  if (!is_interactive_)
    return; // Disable dragging in Subscriber Mode
  if (dragging_left_ || dragging_right_) {
    QPointF pos = event->pos();
    if (dragging_left_) {
      // Calculate new axes values for left stick
      float dx = -(pos.x() - 60) / 20.0f; // Invert X-axis
      float dy = (50 - pos.y()) / 20.0f;
      // Clamp to unit circle
      float mag = std::sqrt(dx * dx + dy * dy);
      if (mag > 1.0f) {
        dx /= mag;
        dy /= mag;
      }
      axes_values_[0] = dx;
      axes_values_[1] = dy;
    } else if (dragging_right_) {
      // Calculate new axes values for right stick
      float dx = -(pos.x() - 140) / 20.0f; // Invert X-axis, horizontal
      float dy = (50 - pos.y()) / 20.0f;   // Y-axis, vertical
      // Clamp to unit circle
      float mag = std::sqrt(dx * dx + dy * dy);
      if (mag > 1.0f) {
        dx /= mag;
        dy /= mag;
      }
      axes_values_[3] = dx;
      axes_values_[4] = dy;
    }
    update(); // Trigger repaint
    Q_EMIT joystickMoved(axes_values_);
  }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent *) {
  if (!is_interactive_)
    return; // Disable dragging in Subscriber Mode
  dragging_left_ = false;
  dragging_right_ = false;
  // Reset sticks to center in Publisher Mode
  axes_values_[0] = 0.0f;
  axes_values_[1] = 0.0f;
  axes_values_[3] = 0.0f;
  axes_values_[4] = 0.0f;
  update(); // Trigger repaint
  Q_EMIT joystickMoved(axes_values_);
}

void JoystickWidget::setInteractive(bool interactive) {
  is_interactive_ = interactive;
}

JoystickRvizPanel::JoystickRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  setupUi();
  update_timer_ = new QTimer(this);
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(updatePanel()));
  update_timer_->start(100); // 10Hz update rate
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

  // Mode toggle checkbox
  mode_checkbox_ =
      new QCheckBox("Publisher Mode (uncheck for Subscriber Mode)");
  mode_checkbox_->setChecked(false); // Start in Subscriber Mode
  main_layout_->addWidget(mode_checkbox_);
  connect(mode_checkbox_, &QCheckBox::toggled, this,
          &JoystickRvizPanel::onModeChanged);

  // Status display
  status_label_ = new QLabel("Status: Waiting for joystick input...");
  main_layout_->addWidget(status_label_);

  // Joystick visualization widget
  joystick_widget_ = new JoystickWidget;
  joystick_widget_->setInteractive(false); // Start in Subscriber Mode
  main_layout_->addWidget(joystick_widget_);
  connect(joystick_widget_, &JoystickWidget::joystickMoved, this,
          &JoystickRvizPanel::publishJoystickState);

  // Buttons layout
  buttons_layout_ = new QHBoxLayout;
  main_layout_->addLayout(buttons_layout_);

  // Control buttons
  button1_ = new QPushButton("Execute");
  connect(button1_, SIGNAL(clicked()), this, SLOT(onButton1Clicked()));
  buttons_layout_->addWidget(button1_);

  button2_ = new QPushButton("Reset");
  connect(button2_, SIGNAL(clicked()), this, SLOT(onButton2Clicked()));
  buttons_layout_->addWidget(button2_);
}

void JoystickRvizPanel::setupROS() {
  node_ = std::make_shared<rclcpp::Node>("my_rviz_panel_node");

  // Create publisher for joystick data
  joy_pub_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

  // Create subscription for joystick data
  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&JoystickRvizPanel::joyCallback, this, std::placeholders::_1));

  // Create publisher for commands
  pub_command_ =
      node_->create_publisher<std_msgs::msg::String>("panel_commands", 10);

  // Spin the node
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_);
  std::thread([executor]() { executor->spin(); }).detach();
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
  if (!mode_checkbox_->isChecked()) { // Only update in Subscriber Mode
    // Update joystick axes (assuming Logitech controller with at least 4 axes)
    for (size_t i = 0; i < std::min(msg->axes.size(), size_t(6)); ++i) {
      joystick_widget_->axes_values_[i] = msg->axes[i];
    }

    last_status_msg_ = "Joystick input received from subscription";
    joystick_widget_->update(); // Trigger repaint

#if VERBOSITY_LEVEL == 4
    // Print joystick message
    std::stringstream ss;
    ss << "Joy message: axes=[";
    for (size_t i = 0; i < msg->axes.size(); ++i) {
      ss << msg->axes[i];
      if (i < msg->axes.size() - 1)
        ss << ", ";
    }
    ss << "], buttons=[";
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      ss << msg->buttons[i];
      if (i < msg->buttons.size() - 1)
        ss << ", ";
    }
    ss << "]";
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
#endif
  }
}

void JoystickRvizPanel::publishJoystickState(const float axes[4]) {
  if (mode_checkbox_->isChecked()) { // Only publish in Publisher Mode
    auto msg = std::make_unique<sensor_msgs::msg::Joy>();
    msg->axes = {-axes[0], axes[1], -axes[2], axes[3]}; // Invert X-axes
    msg->buttons = {0, 0, 0, 0, 0, 0, 0, 0}; // Placeholder, no buttons
    msg->header.stamp = node_->get_clock()->now();
    joy_pub_->publish(std::move(msg));
    last_status_msg_ = "Joystick position published";

    // Print joystick message
    std::stringstream ss;
    ss << "Published joy message: axes=[" << -axes[0] << ", " << axes[1] << ", "
       << -axes[2] << ", " << axes[3] << "], buttons=[0, 0, 0, 0, 0, 0, 0, 0]";
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
  }
}

void JoystickRvizPanel::updatePanel() {
  status_label_->setText(QString("Status: %1").arg(last_status_msg_.c_str()));
}

void JoystickRvizPanel::onModeChanged(bool checked) {
  joystick_widget_->setInteractive(checked);
  last_status_msg_ =
      checked ? "Switched to Publisher Mode" : "Switched to Subscriber Mode";
  if (!checked) {
    // Clear joystick positions in Subscriber Mode
    for (int i = 0; i < 4; ++i) {
      joystick_widget_->axes_values_[i] = 0.0f;
    }
    joystick_widget_->update();
  }
}

void JoystickRvizPanel::onButton1Clicked() {
  last_status_msg_ = "Execute command sent";
  publishCommand("execute");
}

void JoystickRvizPanel::onButton2Clicked() {
  last_status_msg_ = "Reset command sent";
  for (int i = 0; i < 4; ++i) {
    joystick_widget_->axes_values_[i] = 0.0f;
  }
  joystick_widget_->update();
  publishCommand("reset");
  if (mode_checkbox_->isChecked()) {
    publishJoystickState(joystick_widget_->axes_values_);
  }
}

void JoystickRvizPanel::publishCommand(const std::string &command) {
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = command;
  pub_command_->publish(std::move(msg));
}

void JoystickRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);
  config.mapSetValue("publisher_mode", mode_checkbox_->isChecked());
}

void JoystickRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);
  bool publisher_mode = false;
  if (config.mapGetBool("publisher_mode", &publisher_mode)) {
    mode_checkbox_->setChecked(publisher_mode);
    joystick_widget_->setInteractive(publisher_mode);
  }
}

} // namespace joystick_rviz_panel

PLUGINLIB_EXPORT_CLASS(joystick_rviz_panel::JoystickRvizPanel,
                       rviz_common::Panel)
