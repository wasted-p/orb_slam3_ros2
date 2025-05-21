#include "hexapod_rviz_plugins/control_panel.hpp"
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <string>

namespace hexapod_rviz_plugins {

HexapodControlRvizPanel::HexapodControlRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  setupUi();
}

HexapodControlRvizPanel::~HexapodControlRvizPanel() {}

void HexapodControlRvizPanel::onInitialize() { setupROS(); }

void HexapodControlRvizPanel::setupUi() {
  QVBoxLayout *main_layout = new QVBoxLayout;

  // Create leg position table
  leg_table_ = new QTableWidget(6, 4); // 6 legs, 4 columns (name + xyz)
  leg_table_->setHorizontalHeaderLabels({"Leg", "X", "Y", "Z"});
  // leg_table_->verticalHeader()->setVisible(false);

  // Set leg names
  const QStringList leg_names = {"Leg 1", "Leg 2", "Leg 3",
                                 "Leg 4", "Leg 5", "Leg 6"};
  for (int i = 0; i < 6; ++i) {
    // Leg name
    QTableWidgetItem *name_item = new QTableWidgetItem(leg_names[i]);
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    leg_table_->setItem(i, 0, name_item);

    // X, Y, Z spin boxes
    for (int j = 0; j < 3; ++j) {
      QWidget *widget = new QWidget();
      QHBoxLayout *layout = new QHBoxLayout(widget);
      layout->setContentsMargins(0, 0, 0, 0);

      QDoubleSpinBox *spin_box = createSpinBox();
      layout->addWidget(spin_box);
      widget->setLayout(layout);

      leg_table_->setCellWidget(i, j + 1, widget);

      // Connect value changes
      connect(spin_box, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, &HexapodControlRvizPanel::onPoseChanged);
    }
  }
  leg_table_->resizeColumnsToContents();
  main_layout->addWidget(leg_table_);

  // Pose navigation controls
  QHBoxLayout *nav_layout = new QHBoxLayout;
  QPushButton *prev_button = new QPushButton("Previous");
  QPushButton *next_button = new QPushButton("Next");
  nav_layout->addWidget(prev_button);
  nav_layout->addWidget(next_button);
  main_layout->addLayout(nav_layout);

  connect(prev_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onPreviousPoseClicked);
  connect(next_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onNextPoseClicked);

  // Pose saving controls
  QHBoxLayout *save_layout = new QHBoxLayout;
  pose_name_input_ = new QLineEdit();
  pose_name_input_->setPlaceholderText("Pose name");
  QPushButton *save_button = new QPushButton("Save Pose");
  QPushButton *delete_button = new QPushButton("Delete Pose");
  save_layout->addWidget(pose_name_input_);
  save_layout->addWidget(save_button);
  save_layout->addWidget(delete_button);
  main_layout->addLayout(save_layout);

  connect(save_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onSavePoseClicked);
  connect(delete_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onDeletePoseClicked);

  // Pose list
  pose_list_ = new QListWidget();
  main_layout->addWidget(new QLabel("Saved Poses:"));
  main_layout->addWidget(pose_list_);
  connect(pose_list_, &QListWidget::itemClicked, this,
          &HexapodControlRvizPanel::onPoseSelected);

  setLayout(main_layout);
}

QDoubleSpinBox *HexapodControlRvizPanel::createSpinBox() {
  QDoubleSpinBox *spin_box = new QDoubleSpinBox();
  spin_box->setRange(-100.0, 100.0);
  spin_box->setSingleStep(0.1);
  spin_box->setDecimals(3);
  spin_box->setStyleSheet("QDoubleSpinBox { background: transparent; }");
  return spin_box;
}

void HexapodControlRvizPanel::setupROS() {
  node_ = std::make_shared<rclcpp::Node>("hexapod_rviz_panel_node");
  leg_pose_pub_ = node_->create_publisher<hexapod_msgs::msg::LegPose>(
      "hexapod_control/leg_pose/update", 10);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_);
  std::thread([executor]() { executor->spin(); }).detach();
}

void HexapodControlRvizPanel::onPoseChanged() {
  updateCurrentPoseFromUI();
  publishCurrentPose();
}

void HexapodControlRvizPanel::updateCurrentPoseFromUI() {
  if (current_pose_index_ >= 0 &&
      current_pose_index_ < static_cast<int>(poses_.size())) {
    HexapodPose &pose = poses_[current_pose_index_];

    for (int leg = 0; leg < 6; ++leg) {
      QDoubleSpinBox *x_spin = qobject_cast<QDoubleSpinBox *>(
          leg_table_->cellWidget(leg, 1)->layout()->itemAt(0)->widget());
      QDoubleSpinBox *y_spin = qobject_cast<QDoubleSpinBox *>(
          leg_table_->cellWidget(leg, 2)->layout()->itemAt(0)->widget());
      QDoubleSpinBox *z_spin = qobject_cast<QDoubleSpinBox *>(
          leg_table_->cellWidget(leg, 3)->layout()->itemAt(0)->widget());

      pose.legs[leg].x = x_spin->value();
      pose.legs[leg].y = y_spin->value();
      pose.legs[leg].z = z_spin->value();
    }
  }
}

void HexapodControlRvizPanel::updateUIFromCurrentPose() {
  if (current_pose_index_ >= 0 &&
      current_pose_index_ < static_cast<int>(poses_.size())) {
    const HexapodPose &pose = poses_[current_pose_index_];
    pose_name_input_->setText(QString::fromStdString(pose.name));

    for (int leg = 0; leg < 6; ++leg) {
      QDoubleSpinBox *x_spin = qobject_cast<QDoubleSpinBox *>(
          leg_table_->cellWidget(leg, 1)->layout()->itemAt(0)->widget());
      QDoubleSpinBox *y_spin = qobject_cast<QDoubleSpinBox *>(
          leg_table_->cellWidget(leg, 2)->layout()->itemAt(0)->widget());
      QDoubleSpinBox *z_spin = qobject_cast<QDoubleSpinBox *>(
          leg_table_->cellWidget(leg, 3)->layout()->itemAt(0)->widget());

      x_spin->blockSignals(true);
      y_spin->blockSignals(true);
      z_spin->blockSignals(true);

      x_spin->setValue(pose.legs[leg].x);
      y_spin->setValue(pose.legs[leg].y);
      z_spin->setValue(pose.legs[leg].z);

      x_spin->blockSignals(false);
      y_spin->blockSignals(false);
      z_spin->blockSignals(false);
    }
  }
}

void HexapodControlRvizPanel::publishCurrentPose() {
  if (current_pose_index_ >= 0 &&
      current_pose_index_ < static_cast<int>(poses_.size())) {
    hexapod_msgs::msg::LegPose msg;
    // msg.header.stamp = node_->now();
    // msg.header.frame_id = "base_link";

    const HexapodPose &pose = poses_[current_pose_index_];
    RCLCPP_INFO(node_->get_logger(), "Pose=%.2f,%.2f,%.2f", pose.legs[0].x,
                pose.legs[0].y, pose.legs[0].z);

    // for (int i = 0; i < 6; ++i) {
    hexapod_msgs::msg::LegPose leg_pose;
    leg_pose.leg_name = "top_left";
    leg_pose.position.x = pose.legs[0].x;
    leg_pose.position.y = pose.legs[0].y;
    leg_pose.position.z = pose.legs[0].z;
    // }

    leg_pose_pub_->publish(msg);
  }
}

void HexapodControlRvizPanel::updatePoseList() {
  pose_list_->clear();
  for (const auto &pose : poses_) {
    pose_list_->addItem(QString::fromStdString(pose.name));
  }

  if (!poses_.empty()) {
    pose_list_->setCurrentRow(current_pose_index_);
  }
}

void HexapodControlRvizPanel::onSavePoseClicked() {
  std::string name = pose_name_input_->text().toStdString();
  if (name.empty()) {
    name = "Pose " + std::to_string(poses_.size() + 1);
  }

  HexapodPose new_pose;
  new_pose.name = name;

  for (int leg = 0; leg < 6; ++leg) {
    QDoubleSpinBox *x_spin = qobject_cast<QDoubleSpinBox *>(
        leg_table_->cellWidget(leg, 1)->layout()->itemAt(0)->widget());
    QDoubleSpinBox *y_spin = qobject_cast<QDoubleSpinBox *>(
        leg_table_->cellWidget(leg, 2)->layout()->itemAt(0)->widget());
    QDoubleSpinBox *z_spin = qobject_cast<QDoubleSpinBox *>(
        leg_table_->cellWidget(leg, 3)->layout()->itemAt(0)->widget());

    new_pose.legs[leg].x = x_spin->value();
    new_pose.legs[leg].y = y_spin->value();
    new_pose.legs[leg].z = z_spin->value();
  }

  if (current_pose_index_ >= 0 &&
      current_pose_index_ < static_cast<int>(poses_.size())) {
    poses_[current_pose_index_] = new_pose;
  } else {
    poses_.push_back(new_pose);
    current_pose_index_ = poses_.size() - 1;
  }

  updatePoseList();
}

void HexapodControlRvizPanel::onDeletePoseClicked() {
  if (current_pose_index_ >= 0 &&
      current_pose_index_ < static_cast<int>(poses_.size())) {
    poses_.erase(poses_.begin() + current_pose_index_);
    if (poses_.empty()) {
      current_pose_index_ = -1;
    } else if (current_pose_index_ >= static_cast<int>(poses_.size())) {
      current_pose_index_ = poses_.size() - 1;
    }
    updatePoseList();
    updateUIFromCurrentPose();
  }
}

void HexapodControlRvizPanel::onPoseSelected(QListWidgetItem *item) {
  current_pose_index_ = pose_list_->row(item);
  updateUIFromCurrentPose();
  publishCurrentPose();
}

void HexapodControlRvizPanel::onPreviousPoseClicked() {
  if (!poses_.empty()) {
    current_pose_index_ =
        (current_pose_index_ - 1 + poses_.size()) % poses_.size();
    pose_list_->setCurrentRow(current_pose_index_);
    updateUIFromCurrentPose();
    publishCurrentPose();
  }
}

void HexapodControlRvizPanel::onNextPoseClicked() {
  if (!poses_.empty()) {
    current_pose_index_ = (current_pose_index_ + 1) % poses_.size();
    pose_list_->setCurrentRow(current_pose_index_);
    updateUIFromCurrentPose();
    publishCurrentPose();
  }
}

void HexapodControlRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);
  // TODO: Save poses to config
}

void HexapodControlRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);
  // TODO: Load poses from config
}

} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::HexapodControlRvizPanel,
                       rviz_common::Panel)
