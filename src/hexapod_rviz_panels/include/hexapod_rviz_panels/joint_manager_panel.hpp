#ifndef HEXAPOD_RVIZ_PANELS__JOINT_MANAGER_PANEL_HPP_
#define HEXAPOD_RVIZ_PANELS__JOINT_MANAGER_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp> // Added this line for DisplayContext
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace hexapod_rviz_panels
{
class JointManagerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit JointManagerPanel(QWidget* parent = 0);
  ~JointManagerPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  
  QComboBox* controller_selector_;
  QComboBox* topic_selector_;
  QPushButton* save_pose_button_;
  QLabel* label_;
  std::vector<QSpinBox*> joint_spinboxes_;

  void topicCallback(const std_msgs::msg::String& msg);
  void populateTopicSelector();
  void onControllerChanged(int index);
  void updateJointState();
  void buttonActivated();
};

}  // namespace hexapod_rviz_panels

#endif  // HEXAPOD_RVIZ_PANELS__JOINT_MANAGER_PANEL_HPP_
