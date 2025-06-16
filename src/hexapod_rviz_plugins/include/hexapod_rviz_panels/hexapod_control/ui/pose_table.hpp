#ifndef HEXAPOD_RVIZ_PLUGINS_POSE_TABLE_HPP
#define HEXAPOD_RVIZ_PLUGINS_POSE_TABLE_HPP

#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include <hexapod_common/requests.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <map>
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

geometry_msgs::msg::Point point(const double x, const double y, const double z);

namespace hexapod_rviz_plugins {
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
} // namespace hexapod_rviz_plugins

#endif // CONTROL_RVIZ_PANEL_HPP
//
// #ifndef HEXAPOD_RVIZ_PLUGINS_UI_POSE_LIST
// #define HEXAPOD_RVIZ_PLUGINS_UI_POSE_LIST
//
// #include <QListWidget>
// #include <QStringList>
// #include <qapplication.h>
// #include <qinputdialog.h>
// #include <rclcpp/rclcpp.hpp>
// #include <rviz_common/panel.hpp>
//
// namespace hexapod_rviz_plugins {
// class PoseList : public QListWidget {
//   Q_OBJECT
//   // In your class header
// private:
// public:
//   PoseList();
//   void removePose(size_t idx);
//   void moveCurrentPose(int distance = 1);
//   void moveCurrentPoseUp();
//   void moveCurrentPoseDown();
//
// signals:
//   void poseSelected(const size_t idx);
//   void poseMoved(const int from_idx, const int to_idx);
//
// private:
//   void onPoseSelected();
//   void onRenamePose(QListWidgetItem *item);
//   void onItemClicked(QListWidgetItem *item);
// };
// } // namespace hexapod_rviz_plugins
