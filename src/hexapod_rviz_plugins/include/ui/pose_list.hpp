#ifndef HEXAPOD_RVIZ_PLUGINS_UI_POSE_LIST
#define HEXAPOD_RVIZ_PLUGINS_UI_POSE_LIST

#include <QListWidget>
#include <QStringList>
#include <qapplication.h>
#include <qinputdialog.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace hexapod_rviz_plugins {
class PoseList : public QListWidget {
  Q_OBJECT
  // In your class header
private:
public:
  PoseList();
  void removePose(size_t idx);
  void moveCurrentPose(int distance = 1);
  void moveCurrentPoseUp();
  void moveCurrentPoseDown();
  void addPose(std::string name);

signals:
  void poseSelected(const size_t idx);
  void poseMoved(const int from_idx, const int to_idx);

private:
  void onPoseSelected();
  void onRenamePose(QListWidgetItem *item);
  void onItemClicked(QListWidgetItem *item);
};
} // namespace hexapod_rviz_plugins
#endif
