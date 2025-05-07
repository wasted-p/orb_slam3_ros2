#ifndef JOYSTICK_RVIZ_PANEL_HPP
#define JOYSTICK_RVIZ_PANEL_HPP

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

namespace joystick_rviz_panel {

class JoystickWidget : public QWidget {
  Q_OBJECT
public:
  explicit JoystickWidget(QWidget *parent = nullptr);
  float axes_values_[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  void setInteractive(bool interactive);

Q_SIGNALS:
  void joystickMoved(const float axes[4]);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

private:
  bool dragging_left_ = false;
  bool dragging_right_ = false;
  bool is_interactive_ = false;
};

class JoystickRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit JoystickRvizPanel(QWidget *parent = nullptr);
  ~JoystickRvizPanel() override;
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;

protected Q_SLOTS:
  void onButton1Clicked();
  void onButton2Clicked();
  void updatePanel();
  void publishJoystickState(const float axes[4]);
  void onModeChanged(bool checked);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_command_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  QVBoxLayout *main_layout_;
  QHBoxLayout *buttons_layout_;
  QLabel *status_label_;
  QPushButton *button1_;
  QPushButton *button2_;
  QCheckBox *mode_checkbox_;
  JoystickWidget *joystick_widget_;
  QTimer *update_timer_;
  std::string last_status_msg_ = "Waiting for joystick input...";
  pid_t joy_node_pid_ = 0;

  void setupUi();
  void setupROS();
  void launchJoyNode();
  void publishCommand(const std::string &command);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

} // namespace joystick_rviz_panel

#endif // JOYSTICK_RVIZ_PANEL_HPP
