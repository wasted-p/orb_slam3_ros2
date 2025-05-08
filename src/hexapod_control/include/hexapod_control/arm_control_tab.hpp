#ifndef ARM_CONTROL_TAB_HPP
#define ARM_CONTROL_TAB_HPP

#include "hexapod_control/node_manager.hpp"
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ArmControlTab : public QWidget {
  Q_OBJECT

public:
  explicit ArmControlTab(QWidget *parent = nullptr);

private slots:
  void onExecuteButtonClicked();
  void onRandomButtonClicked();
  void onResetButtonClicked();

private:
  void publishJointStates();
  void publishJointGroupCommand();

  // UI components
  QDoubleSpinBox *joint1SpinBox_;
  QDoubleSpinBox *joint2SpinBox_;
  QDoubleSpinBox *joint3SpinBox_;
  QPushButton *executeButton_;
  QPushButton *randomButton_;
  QPushButton *resetButton_;

  // ROS components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_joint_group_command_;

  // Joint names for the arm (for /joint_states)
  const QStringList jointNames_ = {"arm_rotator_joint", "arm_abductor_joint",
                                   "arm_retractor_joint"};

  // Joint angle limits
  const double MIN_ANGLE = -M_PI / 2;
  const double MAX_ANGLE = M_PI / 2;

  // Flag to track if command is executing
  bool is_executing_ = false;
};

#endif // ARM_CONTROL_TAB_HPP
