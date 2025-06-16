
#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <QApplication>
#include <hexapod_common/hexapod.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_common/yaml_utils.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <map>
#include <pluginlib/class_list_macros.hpp>
#include <qboxlayout.h>
#include <qcheckbox.h>
#include <qlist.h>
#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qtablewidget.h>
#include <rclcpp/client.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <string>
#include <vector>

using namespace rviz_common;
using namespace rclcpp;

namespace hexapod_rviz_plugins {
class ArmJointStateControlPanel : public rviz_common::Panel {
  Q_OBJECT
private:
  rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_state_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  std::map<std::string, QDoubleSpinBox *> spinners_;
  rclcpp::Node::SharedPtr node_;
  double step_ = 0.005;
  std::vector<std::string> joint_names;

public:
  explicit ArmJointStateControlPanel(QWidget *parent = nullptr);

  void onStepSizeChanged(double value);

  void onResetClicked();

  void sendJointState();

  ~ArmJointStateControlPanel();

  void jointStateCallback(const sensor_msgs::msg::JointState &joint_state);

  void onInitialize();

  void setupUi();

  void setupROS();
};
} // namespace hexapod_rviz_plugins
