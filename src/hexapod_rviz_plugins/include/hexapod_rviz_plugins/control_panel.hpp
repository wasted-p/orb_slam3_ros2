#ifndef MY_RVIZ_PANEL_HPP
#define MY_RVIZ_PANEL_HPP

#include "hexapod_msgs/srv/get_pose.hpp"
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
#include <hexapod_msgs/msg/command.hpp>
#include <hexapod_msgs/msg/leg_pose.hpp>
#include <hexapod_msgs/msg/pose.hpp>

namespace hexapod_rviz_plugins {

class PoseTable : public QTableWidget {
  Q_OBJECT
  const static int ROW_COUNT = 6;
  const static int COLUMN_COUNT = 4;
  std::map<std::string, std::array<QDoubleSpinBox *, 3>> spinners_;
  void onSpinnerBoxUpdate();
  QDoubleSpinBox *createSpinBox();
signals:
  void legPoseUpdated(std::string row_name, double x, double y, double z);

public:
  PoseTable();
  void updateSpinners(const hexapod_msgs::msg::Pose pose);
};

class HexapodControlRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit HexapodControlRvizPanel(QWidget *parent = nullptr);
  ~HexapodControlRvizPanel() override;
  void onInitialize() override;

protected Q_SLOTS:

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;

  QStringList leg_names_;

  void setupUi();
  void setupROS();
  void legPoseUpdateCallback(const hexapod_msgs::msg::Pose msg);
  void onLegPoseUpdate(std::string leg_name, double x, double y, double z);

  // UI Elements
  PoseTable *pose_table_;
};

} // namespace hexapod_rviz_plugins

#endif // MY_RVIZ_PANEL_HPP
