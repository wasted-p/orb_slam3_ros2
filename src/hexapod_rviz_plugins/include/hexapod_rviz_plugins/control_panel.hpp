#ifndef MY_RVIZ_PANEL_HPP
#define MY_RVIZ_PANEL_HPP

#include "hexapod_msgs/msg/leg_pose.hpp"
#include <hexapod_msgs/msg/leg_pose.hpp>
#include <rclcpp/rclcpp.hpp>
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

namespace hexapod_rviz_plugins {

class HexapodControlRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit HexapodControlRvizPanel(QWidget *parent = nullptr);
  ~HexapodControlRvizPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;

protected Q_SLOTS:
  void onPoseChanged();
  void onSavePoseClicked();
  void onDeletePoseClicked();
  void onPoseSelected(QListWidgetItem *item);
  void onPreviousPoseClicked();
  void onNextPoseClicked();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<hexapod_msgs::msg::LegPose>::SharedPtr leg_pose_pub_;

  void setupUi();
  void setupROS();
  void publishCurrentPose();
  void updateCurrentPoseFromUI();
  void updateUIFromCurrentPose();
  void updatePoseList();

  struct LegPosition {
    double x;
    double y;
    double z;
  };

  struct HexapodPose {
    std::string name;
    std::array<LegPosition, 6> legs;
  };

  std::vector<HexapodPose> poses_;
  int current_pose_index_ = -1;

  // UI Elements
  QTableWidget *leg_table_;
  QListWidget *pose_list_;
  QLineEdit *pose_name_input_;
  QDoubleSpinBox *createSpinBox();
};

} // namespace hexapod_rviz_plugins

#endif // MY_RVIZ_PANEL_HPP
