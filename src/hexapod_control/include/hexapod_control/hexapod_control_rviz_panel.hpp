// my_rviz_panel.hpp
#ifndef HEXAPOD_CONTROL_RVIZ_PLUGIN
#define HEXAPOD_CONTROL_RVIZ_PLUGIN

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QSqlQuery>
#include <QTimer>
#include <QVBoxLayout>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>

#include "hexapod_control/arm_control_tab.hpp"
#include "hexapod_control/commands_tab.hpp"
#include "hexapod_control/node_manager.hpp"
#include "hexapod_control/poses_tab.hpp"
#include "hexapod_control/warehouse_tab.hpp"
#include <QComboBox>
#include <QFileDialog>
#include <QObject>
#include <QPushButton>
#include <QtGlobal>
#include <QtSql/qsqldatabase.h>
#include <cstdio>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>

namespace hexapod_control_rviz_plugin {

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

  void createWarehouse(const char *dbName);

protected Q_SLOTS:

  // Periodic update callback
  void updatePanel();

private:
  // ROS node
  rclcpp::Node::SharedPtr node_;

  mutable QSqlDatabase db;

  // ROS Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_command_;

  // ROS Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;

  // Qt UI elements
  QVBoxLayout *main_layout_;
  // New UI Elements for MoveIt2 Panel
  QTabWidget *main_tab_widget_;
  // MotionPlanning Tab
  PosesTab *poses_tab;
  ArmControlTab *arm_control_tab;
  CommandsTab *commands_tab;

  // Helper methods
  void setupUi();
  void setupROS();
  void subscribeToTopics();

  // Callback for status messages
  void statusCallback(const std_msgs::msg::String::SharedPtr msg);
};

} // namespace hexapod_control_rviz_plugin

#endif // HEXAPOD_CONTROL_RVIZ_PLUGIN
