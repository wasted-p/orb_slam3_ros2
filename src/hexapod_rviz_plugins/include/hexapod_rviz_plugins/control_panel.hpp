#ifndef CONTROL_RVIZ_PANEL_HPP
#define CONTROL_RVIZ_PANEL_HPP

#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include <hexapod_control/requests.hpp>
#include <hexapod_control/ros_constants.hpp>
#include <map>
#include <qcheckbox.h>
#include <qlist.h>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rviz_common/panel.hpp>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <hexapod_msgs/msg/pose.hpp>

namespace hexapod_rviz_plugins {

geometry_msgs::msg::Point point(const double x, const double y, const double z);

class PoseTable : public QTableWidget {
  Q_OBJECT
  const static int ROW_COUNT = 6;
  const static int COLUMN_COUNT = 4;

  std::map<std::string, std::array<QDoubleSpinBox *, 3>> spinners_;
  void onSpinnerBoxUpdate();
  QDoubleSpinBox *createSpinBox();
  double step_;
signals:
  void legPoseUpdated(std::string row_name, double x, double y, double z);

public:
  PoseTable();

  void setStep(double step);
  void setRowValue(std::string leg_name, geometry_msgs::msg::Point position);
  geometry_msgs::msg::Point getRowValue(std::string);
  hexapod_msgs::msg::Pose getPose();
  void setPose(const hexapod_msgs::msg::Pose &pose);
};

class HexapodControlRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit HexapodControlRvizPanel(QWidget *parent = nullptr);
  ~HexapodControlRvizPanel() override;
  void onInitialize() override;

protected Q_SLOTS:
  void setRelativeMode(bool checked);
  void onStepSizeChanged(double value);
  void onResetClicked();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr set_pose_client_;

  bool relative = true;

  std::map<std::string, geometry_msgs::msg::Point> initial_pose_;

  // NOTE: Populate these values dynamically from the hexapod_pose node
  void setupUi();
  void setupROS();
  void legPoseUpdateCallback(const hexapod_msgs::msg::Pose msg);
  void onLegPoseUpdate(std::string leg_name, double x, double y, double z);

  // UI Elements
  PoseTable *pose_table_;
};

} // namespace hexapod_rviz_plugins

#endif // CONTROL_RVIZ_PANEL_HPP
