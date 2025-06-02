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
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <unistd.h>

namespace hexapod_rviz_panels {

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
  // launchJoyNode();
}

void JoystickRvizPanel::setupUi() {
  main_layout_ = new QHBoxLayout;
  setLayout(main_layout_);

  // === LEFT SIDE: Stack image + joystick ===
  // QHBoxLayout
  QVBoxLayout *left_layout = new QVBoxLayout;
  // main_layout_->addLayout(left_layout);

  // Status display
  status_label_ = new QLabel("Status: Waiting for joystick input...");
  left_layout->addWidget(status_label_);

  // Stacked layout: image + joystick widget
  QStackedLayout *stacked_layout = new QStackedLayout;
  QWidget *stacked_container = new QWidget;
  stacked_container->setLayout(stacked_layout);
  main_layout_->addWidget(stacked_container, /*stretch=*/1);

  // Controller background image
  QLabel *background_image = new QLabel;
  QPixmap original(":/assets/controller.png");
  QPixmap scaled =
      original.scaled(100, 75, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  background_image->setPixmap(scaled);
  background_image->setScaledContents(true); // Resize to fit
  background_image->setSizePolicy(QSizePolicy::Expanding,
                                  QSizePolicy::Expanding);
  stacked_layout->addWidget(background_image);

  // Joystick visualization widget (overlaid)
  joystick_left_ = new JoystickWidget;
  joystick_left_->setAttribute(Qt::WA_TransparentForMouseEvents);
  joystick_left_->setStyleSheet("background: transparent;");
  joystick_left_->setInteractive(false); // Start in Subscriber Mode
  // stacked_layout->addWidget(joystick_left_);

  connect(joystick_left_, &JoystickWidget::joystickMoved, this,
          &JoystickRvizPanel::publishJoystickState);

  // === RIGHT SIDE: Diamond-shaped buttons ===
  QVBoxLayout *right_layout = new QVBoxLayout;
  // stacked_layout->addItem(right_layout);

  QGridLayout *diamond_layout = new QGridLayout;
  right_layout->addLayout(diamond_layout);
  right_layout->addStretch();

  // Create 4 buttons in diamond layout
  QPushButton *up_button = new QPushButton("▲");
  QPushButton *down_button = new QPushButton("▼");
  QPushButton *left_button = new QPushButton("◀");
  QPushButton *right_button = new QPushButton("▶");

  diamond_layout->addWidget(up_button, 0, 1);
  diamond_layout->addWidget(left_button, 1, 0);
  diamond_layout->addWidget(right_button, 1, 2);
  diamond_layout->addWidget(down_button, 2, 1);

  // Optional: central button (disabled)
  QLabel *center = new QLabel;
  diamond_layout->addWidget(center, 1, 1);
  center->setFixedSize(20, 20);
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
  if (false) { // Only update in Subscriber Mode
    // Update joystick axes (assuming Logitech controller with at least 4 axes)
    for (size_t i = 0; i < std::min(msg->axes.size(), size_t(6)); ++i) {
      joystick_left_->axes_values_[i] = msg->axes[i];
    }

    last_status_msg_ = "Joystick input received from subscription";
    joystick_left_->update(); // Trigger repaint
  }
}

void JoystickRvizPanel::publishJoystickState(const float axes[4]) {
  if (false) { // Only publish in Publisher Mode
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

void JoystickRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);
}

void JoystickRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);
}

} // namespace hexapod_rviz_panels

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_panels::JoystickRvizPanel,
                       rviz_common::Panel)
