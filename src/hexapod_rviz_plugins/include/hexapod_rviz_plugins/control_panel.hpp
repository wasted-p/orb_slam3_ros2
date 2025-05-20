#ifndef HEXAPOD_RVIZ_PANEL_HPP
#define HEXAPOD_RVIZ_PANEL_HPP

#include "hexapod_msgs/msg/leg_pose.hpp"
#include <QDoubleSpinBox>
#include <QHeaderView>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <hexapod_msgs/msg/leg_pose.hpp>

#include <qboxlayout.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Eigen>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>

namespace hexapod_rviz_plugins {

class CoordinateInput : public QWidget {
  Q_OBJECT
public:
  explicit CoordinateInput(QWidget *parent = nullptr);

  void setValues(double x, double y, double z);
  void getValues(double &x, double &y, double &z) const;
  QDoubleSpinBox *xSpinner();
  QDoubleSpinBox *zSpinner();
  QDoubleSpinBox *ySpinner();

private:
  QDoubleSpinBox *x_spin_;
  QDoubleSpinBox *y_spin_;
  QDoubleSpinBox *z_spin_;
};

class HexapodControlRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit HexapodControlRvizPanel(QWidget *parent = nullptr);
  ~HexapodControlRvizPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;

private Q_SLOTS:
  void updatePanel();
  void onAddPoseClicked();
  void onRemovePoseClicked();
  void onSendPoseClicked(int pose_index);

private:
  void setupUi();
  void setupPosesTab();
  void setupSettingsTab();
  void setupROS();
  void subscribeToTopics();
  void publishPose(int pose_index);
  void addPoseColumn();
  void removePoseColumn(int index);
  void updateMarkerPosition(std::string name, Eigen::Vector3d position);

  void statusCallback(const std_msgs::msg::String::SharedPtr msg);

  // UI Elements
  QVBoxLayout *main_layout_;
  QTabWidget *tab_widget_;
  QTableWidget *pose_table_;
  QLabel *status_label_;

  // Buttons
  QPushButton *add_pose_button_;
  QPushButton *remove_pose_button_;

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<hexapod_msgs::msg::LegPose>::SharedPtr pub_poses_;

  // Other members
  QTimer *update_timer_;
  std::string last_status_msg_;
  int current_pose_count_;
};

} // namespace hexapod_rviz_plugins

#endif // HEXAPOD_CONTROL_RVIZ_PANEL_HPP
