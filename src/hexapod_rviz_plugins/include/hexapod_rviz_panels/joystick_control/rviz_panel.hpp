#ifndef JOYSTICK_RVIZ_PANEL_HPP
#define JOYSTICK_RVIZ_PANEL_HPP

#include <qboxlayout.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <hexapod_msgs/msg/motion.hpp>
#include <hexapod_rviz_panels/joystick_control/ui/joystick.hpp>

namespace hexapod_rviz_plugins {

class JoystickRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit JoystickRvizPanel(QWidget *parent = nullptr);
  ~JoystickRvizPanel() override;
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;

protected Q_SLOTS:
  void updatePanel();
  void publishJoystickState(const float axis_values[6]);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<hexapod_msgs::msg::Motion>::SharedPtr motion_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  QVBoxLayout *main_layout_;
  QTimer *update_timer_;
  pid_t joy_node_pid_ = 0;
  bool publisher_mode_ = false;

  ControllerWidget *controller;

  void setupUi();
  void setupROS();
  void publishCommand(const std::string &command);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void setPublishing(bool state);
};

} // namespace hexapod_rviz_plugins

#endif // JOYSTICK_RVIZ_PANEL_HPP
