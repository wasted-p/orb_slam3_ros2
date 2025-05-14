#ifndef MY_RVIZ_PANEL_HPP
#define MY_RVIZ_PANEL_HPP

#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
// #include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <yaml-cpp/yaml.h>

namespace hexapod_control_rviz_plugin {

class HexapodControlRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit HexapodControlRvizPanel(QWidget *parent = nullptr);
  ~HexapodControlRvizPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;

protected Q_SLOTS:
  void updatePanel();

  void onAddRow();
  void onDeleteRow();
  void onMoveRowUp();
  void onMoveRowDown();
  void onExecuteGait();
  void onSaveGait();
  void onLoadGait();

private:
  // ROS
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_pub_;

  // Status tracking
  std::string last_status_msg_;

  // UI
  QVBoxLayout *main_layout_;
  QLabel *status_label_;
  QTableWidget *gait_table_;
  QTimer *update_timer_;

  QStringList positions = {"top_left",    "mid_left",
                           "bottom_left", "top_right",
                           "mid_right",   "bottom_right"}; // Gait Editor Table

  // Core setup
  void setupUi();
  void setupROS();
  void subscribeToTopics();

  // Communication
  void statusCallback(const std_msgs::msg::String::SharedPtr msg);

  // Helpers
  void swapRows(int row1, int row2);
  void updateTableAndMarkers();
};

} // namespace hexapod_control_rviz_plugin

#endif // MY_RVIZ_PANEL_HPP
