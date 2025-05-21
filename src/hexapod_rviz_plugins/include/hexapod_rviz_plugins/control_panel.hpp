// my_rviz_panel.hpp
#ifndef MY_RVIZ_PANEL_HPP
#define MY_RVIZ_PANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace hexapod_rviz_plugins {

class HexapodControlRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  // Constructor
  explicit HexapodControlRvizPanel(QWidget *parent = nullptr);

  // Destructor
  ~HexapodControlRvizPanel() override;

  // Override from rviz_common::Panel
  void onInitialize() override;

  // Override from rviz_common::Panel
  void save(rviz_common::Config config) const override;

  // Override from rviz_common::Panel
  void load(const rviz_common::Config &config) override;

protected Q_SLOTS:
  // Button click handlers
  void onButton1Clicked();
  void onButton2Clicked();

  // Periodic update callback
  void updatePanel();

private:
  // ROS node
  rclcpp::Node::SharedPtr node_;

  // ROS Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_command_;

  // ROS Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;

  // Qt UI elements
  QVBoxLayout *main_layout_;
  QHBoxLayout *buttons_layout_;
  QLabel *status_label_;
  QPushButton *button1_;
  QPushButton *button2_;
  QLineEdit *text_input_;
  QComboBox *option_combo_;

  // Update timer
  QTimer *update_timer_;

  // Track status messages
  std::string last_status_msg_;

  // Helper methods
  void setupUi();
  void setupROS();
  void subscribeToTopics();
  void publishCommand(const std::string &command);

  // Callback for status messages
  void statusCallback(const std_msgs::msg::String::SharedPtr msg);
};

} // namespace hexapod_rviz_plugins

#endif // MY_RVIZ_PANEL_HPP
