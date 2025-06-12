#ifndef HEXAPOD_RVIZ_PLUGINS_MOTION_EDITOR_HPP
#define HEXAPOD_RVIZ_PLUGINS_MOTION_EDITOR_HPP

#include "hexapod_msgs/srv/get_pose.hpp"
#include "hexapod_msgs/srv/set_marker_array.hpp"
#include <QListWidget>
#include <QStringList>
#include <map>
#include <qcombobox.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <ui/pose_list.hpp>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>

// TODO: Edit multiple actinos in same file
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/set_marker_array.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include "ui/pose_list.hpp"
#include <QApplication>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <qevent.h>
#include <qfiledialog.h>
#include <qlist.h>
#include <qlistwidget.h>
#include <qobject.h>
#include <qobjectdefs.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qvariant.h>
#include <rclcpp/client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <yaml-cpp/yaml.h>

namespace hexapod_rviz_plugins {

struct Motion {
  std::string name;
  std::string category;
  std::string type;
  double duration;
  std::vector<hexapod_msgs::msg::Pose> poses;
};

class MotionEditorRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  MotionEditorRvizPanel(QWidget *parent = nullptr);
  ~MotionEditorRvizPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onAddPose();
  void onDeletePose();

  void onRenamePose(QListWidgetItem *item);
  void onPoseMoved(const int from_idx, const int to_idx);

private:
  void setSelectedMotion(std::string name);
  void setCurrentPose(const size_t idx);
  void setupUi();
  void setupROS();

  hexapod_msgs::msg::Pose getInitialPose();

  void onPoseUpdate(hexapod_msgs::msg::Pose pose);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;
  double yaw_ = 0.0;
  double effort = 1;

  QComboBox *motion_combo_box_;
  PoseList *pose_list_widget_;
  QStringList leg_names_;
  std::map<std::string, Motion> motions_;
  hexapod_msgs::msg::Pose initial_pose;

  size_t current_pose = -1;
  int created_poses_count_ = 0;

  rclcpp::Client<hexapod_msgs::srv::SetMarkerArray>::SharedPtr
      set_marker_array_client_;
  rclcpp::Client<hexapod_msgs::srv::GetPose>::SharedPtr get_pose_client_;
  rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr set_pose_client_;

  std::string selected_motion_;
  rclcpp::TimerBase::SharedPtr timer_;
  Motion &selectedMotion();
  Motion transformedMotion();
  hexapod_msgs::msg::Pose &selectedPose();
};

} // namespace hexapod_rviz_plugins

#endif // HEXAPOD_RVIZ_PLUGINS_MOTION_EDITOR_HPP
