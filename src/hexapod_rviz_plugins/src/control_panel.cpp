#include "hexapod_rviz_plugins/control_panel.hpp"
#include "hexapod_msgs/msg/leg_pose.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QScrollArea>
#include <QScrollBar>
#include <QVBoxLayout>
#include <hexapod_msgs/msg/leg_pose.hpp>
#include <qchar.h>
#include <qspinbox.h>
#include <rclcpp/logging.hpp>

namespace hexapod_rviz_plugins {

QDoubleSpinBox *CoordinateInput::xSpinner() { return x_spin_; }
QDoubleSpinBox *CoordinateInput::ySpinner() { return y_spin_; }
QDoubleSpinBox *CoordinateInput::zSpinner() { return z_spin_; }

CoordinateInput::CoordinateInput(QWidget *parent) : QWidget(parent) {
  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(2);

  x_spin_ = new QDoubleSpinBox();
  y_spin_ = new QDoubleSpinBox();
  z_spin_ = new QDoubleSpinBox();

  // Configure spin boxes
  QDoubleSpinBox *spins[] = {x_spin_, y_spin_, z_spin_};

  for (int i = 0; i < 3; ++i) {
    spins[i]->setRange(-10.0, 10.0);
    spins[i]->setSingleStep(0.01);
    spins[i]->setDecimals(3);
    spins[i]->setButtonSymbols(QAbstractSpinBox::NoButtons);
    spins[i]->setStyleSheet(
        "QDoubleSpinBox { border: none; background: transparent; }");

    layout->addWidget(spins[i]);

    if (i < 2) {
      QFrame *divider = new QFrame();
      divider->setFrameShape(QFrame::VLine);
      divider->setFrameShadow(QFrame::Sunken);
      layout->addWidget(divider);
    }
  }
}

void CoordinateInput::setValues(double x, double y, double z) {
  x_spin_->setValue(x);
  y_spin_->setValue(y);
  z_spin_->setValue(z);
}

void CoordinateInput::getValues(double &x, double &y, double &z) const {
  x = x_spin_->value();
  y = y_spin_->value();
  z = z_spin_->value();
}

HexapodControlRvizPanel::HexapodControlRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent), last_status_msg_("No status received yet"),
      current_pose_count_(0) {
  setupUi();

  update_timer_ = new QTimer(this);
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(updatePanel()));
  update_timer_->start(100);
}

HexapodControlRvizPanel::~HexapodControlRvizPanel() {
  if (update_timer_) {
    update_timer_->stop();
  }
}

void HexapodControlRvizPanel::onInitialize() { setupROS(); }

void HexapodControlRvizPanel::setupUi() {
  main_layout_ = new QVBoxLayout;
  setLayout(main_layout_);

  tab_widget_ = new QTabWidget();
  main_layout_->addWidget(tab_widget_);

  setupPosesTab();
  setupSettingsTab();

  status_label_ = new QLabel("Status: Initializing...");
  main_layout_->addWidget(status_label_);
}

void HexapodControlRvizPanel::setupPosesTab() {
  QWidget *poses_tab = new QWidget();
  tab_widget_->addTab(poses_tab, "Poses");

  QVBoxLayout *poses_layout = new QVBoxLayout(poses_tab);

  // Create table for poses
  // Create table for poses
  pose_table_ = new QTableWidget(6, 0); // 6 legs, dynamic columns for poses

  // Set fixed column width
  pose_table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  pose_table_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  pose_table_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

  // Create scroll area
  QScrollArea *scroll_area = new QScrollArea();
  scroll_area->setWidgetResizable(true);
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  // Container widget for table
  QWidget *table_container = new QWidget();
  QVBoxLayout *container_layout = new QVBoxLayout(table_container);
  container_layout->addWidget(pose_table_);
  container_layout->setContentsMargins(0, 0, 0, 0);

  scroll_area->setWidget(table_container);
  poses_layout->addWidget(scroll_area);

  // Set table properties
  pose_table_->setVerticalHeaderLabels(
      {"Leg 1", "Leg 2", "Leg 3", "Leg 4", "Leg 5", "Leg 6"});
  pose_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  pose_table_->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  pose_table_->setSelectionMode(QAbstractItemView::NoSelection);

  // Add initial pose
  addPoseColumn();

  // Add pose management buttons
  QHBoxLayout *button_layout = new QHBoxLayout();
  poses_layout->addLayout(button_layout);

  add_pose_button_ = new QPushButton("Add Pose");
  connect(add_pose_button_, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onAddPoseClicked);
  button_layout->addWidget(add_pose_button_);

  remove_pose_button_ = new QPushButton("Remove Pose");
  connect(remove_pose_button_, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onRemovePoseClicked);
  button_layout->addWidget(remove_pose_button_);
}

void HexapodControlRvizPanel::setupSettingsTab() {
  QWidget *settings_tab = new QWidget();
  tab_widget_->addTab(settings_tab, "Settings");

  QVBoxLayout *settings_layout = new QVBoxLayout(settings_tab);
  settings_layout->addWidget(new QLabel("Hexapod Control Settings"));
  // Add more settings as needed
}

void HexapodControlRvizPanel::addPoseColumn() {
  int new_col = current_pose_count_;

  pose_table_->insertColumn(new_col);
  pose_table_->setColumnWidth(new_col, 150); // or whatever fixed size
  pose_table_->horizontalHeader()->setSectionResizeMode(new_col,
                                                        QHeaderView::Fixed);
  pose_table_->setHorizontalHeaderItem(
      new_col, new QTableWidgetItem(QString("Pose %1").arg(new_col + 1)));

  const int LEG_COUNT = 6;
  std::string LEG_POSITION[LEG_COUNT] = {
      "top_left",  "mid_left",  "bottom_left",
      "top_right", "mid_right", "bottom_right",
  };

  // Add coordinate inputs for each leg
  for (int i = 0; i < LEG_COUNT; ++i) {
    CoordinateInput *coord_input = new CoordinateInput();
    pose_table_->setCellWidget(i, new_col, coord_input);

    // Connect send signal for this pose
    connect(coord_input->xSpinner(),
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this, new_col](double) {
              updateMarkerPosition("top_left", {0, 0, 0});
            });
    connect(coord_input->ySpinner(),
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this, new_col](double) {
              updateMarkerPosition("top_left", {0, 0, 0});
            });
    connect(coord_input->zSpinner(),
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this, new_col](double) { onSendPoseClicked(new_col); });
  }

  current_pose_count_++;
}

void HexapodControlRvizPanel::updateMarkerPosition(std::string name,
                                                   Eigen::Vector3d position) {
  auto msg = std::make_unique<hexapod_msgs::msg::LegPose>();
  msg->leg_name = name;
  msg->position.x = position.x();
  msg->position.y = position.y();
  msg->position.z = position.z();
  pub_poses_->publish(std::move(msg));
}

void HexapodControlRvizPanel::removePoseColumn(int index) {
  if (current_pose_count_ <= 1)
    return; // Keep at least one pose

  pose_table_->removeColumn(index);
  current_pose_count_--;

  // Renumber remaining poses
  for (int i = 0; i < current_pose_count_; ++i) {
    pose_table_->horizontalHeaderItem(i)->setText(
        QString("Pose %1").arg(i + 1));
  }
}

void HexapodControlRvizPanel::onAddPoseClicked() { addPoseColumn(); }

void HexapodControlRvizPanel::onRemovePoseClicked() {
  removePoseColumn(current_pose_count_ - 1);
}

void HexapodControlRvizPanel::onSendPoseClicked(int pose_index) {
  publishPose(pose_index);
}

void HexapodControlRvizPanel::publishPose(int pose_index) {
  std::string pose_data = "Pose " + std::to_string(pose_index + 1) + ": ";

  for (int leg = 0; leg < 6; ++leg) {
    CoordinateInput *coord_input = qobject_cast<CoordinateInput *>(
        pose_table_->cellWidget(leg, pose_index));

    if (coord_input) {
      double x, y, z;
      coord_input->getValues(x, y, z);
      pose_data += QString("Leg%1:(%2,%3,%4) ")
                       .arg(leg + 1)
                       .arg(x)
                       .arg(y)
                       .arg(z)
                       .toStdString();
    }
  }
}

void HexapodControlRvizPanel::setupROS() {
  node_ = std::make_shared<rclcpp::Node>("hexapod_control_rviz_panel_node");
  pub_poses_ = node_->create_publisher<hexapod_msgs::msg::LegPose>(
      "/hexapod_control/leg_pose/update", 10);

  subscribeToTopics();

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_);
  std::thread([executor]() { executor->spin(); }).detach();
}

void HexapodControlRvizPanel::subscribeToTopics() {}

void HexapodControlRvizPanel::statusCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  last_status_msg_ = msg->data;
}

void HexapodControlRvizPanel::updatePanel() {
  status_label_->setText(QString("Status: %1").arg(last_status_msg_.c_str()));
}

void HexapodControlRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);

  // Save all poses
  config.mapSetValue("pose_count", current_pose_count_);

  for (int pose = 0; pose < current_pose_count_; ++pose) {
    for (int leg = 0; leg < 6; ++leg) {
      CoordinateInput *coord_input =
          qobject_cast<CoordinateInput *>(pose_table_->cellWidget(leg, pose));

      if (coord_input) {
        double x, y, z;
        coord_input->getValues(x, y, z);

        std::string key =
            QString("pose%1_leg%2").arg(pose).arg(leg).toStdString();
        config.mapSetValue(QString::fromStdString(key + "_x"), x);
        config.mapSetValue(QString::fromStdString(key + "_y"), y);
        config.mapSetValue(QString::fromStdString(key + "_z"), z);
      }
    }
  }
}

void HexapodControlRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);

  // Load pose count
  int saved_pose_count;
  if (config.mapGetInt("pose_count", &saved_pose_count)) {
    // Remove default pose if needed
    while (current_pose_count_ > 0) {
      removePoseColumn(0);
    }

    // Add saved poses
    for (int i = 0; i < saved_pose_count; ++i) {
      addPoseColumn();
    }

    // Load values
    for (int pose = 0; pose < saved_pose_count; ++pose) {
      for (int leg = 0; leg < 6; ++leg) {
        std::string key =
            QString("pose%1_leg%2").arg(pose).arg(leg).toStdString();

        float x, y, z;
        if (config.mapGetFloat(QString::fromStdString(key + "_z"), &z) &&
            config.mapGetFloat(QString::fromStdString(key + "_z"), &z) &&
            config.mapGetFloat(QString::fromStdString(key + "_z"), &z)) {

          CoordinateInput *coord_input = qobject_cast<CoordinateInput *>(
              pose_table_->cellWidget(leg, pose));

          if (coord_input) {
            coord_input->setValues(x, y, z);
          }
        }
      }
    }
  }
}

} // namespace hexapod_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::HexapodControlRvizPanel,
                       rviz_common::Panel)
