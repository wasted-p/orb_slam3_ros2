// my_rviz_panel.cpp
#include "rviz_plugin/hexapod_control_panel.hpp"
#include <fstream>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace hexapod_control_rviz_plugin {

#include <QHBoxLayout>
#include <QLineEdit>
#include <QRegExp>
#include <QRegExpValidator>
#include <QWidget>

class PointInputLineEdit : public QLineEdit {
public:
  explicit PointInputLineEdit(QWidget *parent = nullptr) : QLineEdit(parent) {
    setPlaceholderText("x, y, z");
    setAlignment(Qt::AlignCenter);
    setStyleSheet("QLineEdit { border: none; }");
    setText("0.0, 0.0, 0.0");

    QRegExp rx(
        R"(^\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*$)");
    setValidator(new QRegExpValidator(rx, this));
  }

  void focusOutEvent(QFocusEvent *event) override {
    QLineEdit::focusOutEvent(event);

    // Attempt to find QTableWidget parent and emit cellChanged manually
    QWidget *w = this;
    while (w && !qobject_cast<QTableWidget *>(w)) {
      w = w->parentWidget();
    }

    if (auto *table = qobject_cast<QTableWidget *>(w)) {
      for (int row = 0; row < table->rowCount(); ++row) {
        for (int col = 0; col < table->columnCount(); ++col) {
          if (table->cellWidget(row, col) == this) {
            emit table->cellChanged(row, col);
            return;
          }
        }
      }
    }
  }

  bool getPoint(double &x, double &y, double &z) const {
    QStringList parts = text().trimmed().split(',', Qt::SkipEmptyParts);
    if (parts.size() != 3)
      return false;

    bool ok1, ok2, ok3;
    x = parts[0].trimmed().toDouble(&ok1);
    y = parts[1].trimmed().toDouble(&ok2);
    z = parts[2].trimmed().toDouble(&ok3);
    return ok1 && ok2 && ok3;
  }

  void setPoint(double x, double y, double z) {
    setText(QString("%1, %2, %3")
                .arg(x, 0, 'f', 2)
                .arg(y, 0, 'f', 2)
                .arg(z, 0, 'f', 2));
  }
};

HexapodControlRvizPanel::HexapodControlRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent), last_status_msg_("No status received yet") {
  // Set up the UI elements
  setupUi();

  // Create a timer for periodic updates
  update_timer_ = new QTimer(this);
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(updatePanel()));

  // Inside your HexapodControlRvizPanel constructor or setupUI method
  // Inside your HexapodControlRvizPanel constructor or setupUI method
  connect(gait_table_, &QTableWidget::cellChanged, this,
          [this](int row, int column) {
            // Extract updated value from the table for the specific row and
            // column
            // double updated_value = getTableDoubleValue(row, column);

            // Update the marker with the new value
            // updateMarker(row, column, updated_value);

            // After updating the marker, re-publish all the markers
            updateTableAndMarkers();
          });
  update_timer_->start(100); // 10Hz update rate
}

HexapodControlRvizPanel::~HexapodControlRvizPanel() {
  if (update_timer_) {
    update_timer_->stop();
  }
  // Publishers and subscribers will be cleaned up automatically
}

void HexapodControlRvizPanel::onInitialize() {
  // Create ROS node and set up ROS communications
  setupROS();
}

void HexapodControlRvizPanel::setupUi() {
  // Main layout
  main_layout_ = new QVBoxLayout;
  setLayout(main_layout_);

  // Status display
  status_label_ = new QLabel("Status: Initializing...");
  main_layout_->addWidget(status_label_);

  gait_table_ = new QTableWidget(0, 6, this);
  gait_table_->setHorizontalHeaderLabels(positions);
  // gait_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  main_layout_->addWidget(gait_table_);

  // Table control buttons
  QHBoxLayout *table_buttons_layout = new QHBoxLayout;
  main_layout_->addLayout(table_buttons_layout);

  auto add_button = new QPushButton("Add Row");
  connect(add_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onAddRow);
  table_buttons_layout->addWidget(add_button);

  auto delete_button = new QPushButton("Delete Row");
  connect(delete_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onDeleteRow);
  table_buttons_layout->addWidget(delete_button);

  auto move_up_button = new QPushButton;
  move_up_button->setIcon(QIcon::fromTheme("go-up"));
  connect(move_up_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onMoveRowUp);
  table_buttons_layout->addWidget(move_up_button);

  auto move_down_button = new QPushButton;
  move_down_button->setIcon(QIcon::fromTheme("go-down"));
  connect(move_down_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onMoveRowDown);
  table_buttons_layout->addWidget(move_down_button);

  // Execute/Save/Load buttons
  QHBoxLayout *exec_layout = new QHBoxLayout;
  main_layout_->addLayout(exec_layout);

  auto execute_button = new QPushButton("Execute Gait");
  connect(execute_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onExecuteGait);
  exec_layout->addWidget(execute_button);

  auto save_button = new QPushButton("Save");
  connect(save_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onSaveGait);
  exec_layout->addWidget(save_button);

  auto load_button = new QPushButton("Load");
  connect(load_button, &QPushButton::clicked, this,
          &HexapodControlRvizPanel::onLoadGait);
  exec_layout->addWidget(load_button);
}

void HexapodControlRvizPanel::setupROS() {
  // Create node with unique name
  node_ = std::make_shared<rclcpp::Node>("my_rviz_panel_node");
  marker_array_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "point_marker_array", 10);

  // Set up subscriptions
  subscribeToTopics();

  // Spin the node in a separate thread
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_);
  std::thread([executor]() { executor->spin(); }).detach();
}

void HexapodControlRvizPanel::subscribeToTopics() {
  // Subscribe to status updates
}

void HexapodControlRvizPanel::statusCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  last_status_msg_ = msg->data;
}

void HexapodControlRvizPanel::updatePanel() {
  // Update the status display
  status_label_->setText(QString("Status: %1").arg(last_status_msg_.c_str()));

  // Perform any other periodic updates here
}

void HexapodControlRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);

  // Save UI state to config
  // config.mapSetValue("text", text_input_->text().toStdString().c_str());
  // config.mapSetValue("option", option_combo_->currentIndex());
}

void HexapodControlRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);

  // Load UI state from config
  // QString text;
  // if (config.mapGetString("text", &text)) {
  //   text_input_->setText(text);
  // }
  //
  // int option_index;
  // if (config.mapGetInt("option", &option_index)) {
  //   option_combo_->setCurrentIndex(option_index);
  // }
}

void HexapodControlRvizPanel::onExecuteGait() {
  // No-op for now
}
void HexapodControlRvizPanel::onSaveGait() {
  QString name = QInputDialog::getText(this, "Save Gait", "Enter gait name:");
  if (name.isEmpty())
    return;

  QString file_path = QFileDialog::getSaveFileName(this, "Save YAML", "",
                                                   "YAML files (*.yaml)");
  if (file_path.isEmpty())
    return;

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "gait_name" << YAML::Value << name.toStdString();
  out << YAML::Key << "poses" << YAML::Value << YAML::BeginSeq;

  for (int row = 0; row < gait_table_->rowCount(); ++row) {
    out << YAML::BeginSeq;
    for (int col = 0; col < 6; ++col) {
      auto widget = gait_table_->cellWidget(row, col);
      auto layout = qobject_cast<QHBoxLayout *>(widget->layout());
      double x =
          qobject_cast<QDoubleSpinBox *>(layout->itemAt(0)->widget())->value();
      double y =
          qobject_cast<QDoubleSpinBox *>(layout->itemAt(1)->widget())->value();
      double z =
          qobject_cast<QDoubleSpinBox *>(layout->itemAt(2)->widget())->value();
      out << YAML::Flow << YAML::BeginSeq << x << y << z << YAML::EndSeq;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndSeq << YAML::EndMap;

  std::ofstream fout(file_path.toStdString());
  fout << out.c_str();
}

void HexapodControlRvizPanel::onLoadGait() {
  QString file_path = QFileDialog::getOpenFileName(this, "Open YAML", "",
                                                   "YAML files (*.yaml)");
  if (file_path.isEmpty())
    return;

  YAML::Node doc = YAML::LoadFile(file_path.toStdString());
  if (!doc["poses"])
    return;

  int row = gait_table_->rowCount();
  gait_table_->insertRow(row);

  for (int leg = 0; leg < 6; ++leg) {
    PointInputLineEdit *line_edit = new PointInputLineEdit();
    gait_table_->setCellWidget(row, leg, line_edit);
  }

  // gait_table_->setRowCount(0);
  // for (const auto &pose : doc["poses"]) {
  //   int row = gait_table_->rowCount();
  //   gait_table_->insertRow(row);
  //   for (int col = 0; col < 6; ++col) {
  //     QWidget *cell = createXYZWidget();
  //     auto layout = qobject_cast<QHBoxLayout *>(cell->layout());
  //     auto leg = pose[col];
  //     qobject_cast<QDoubleSpinBox *>(layout->itemAt(0)->widget())
  //         ->setValue(leg[0].as<double>());
  //     qobject_cast<QDoubleSpinBox *>(layout->itemAt(1)->widget())
  //         ->setValue(leg[1].as<double>());
  //     qobject_cast<QDoubleSpinBox *>(layout->itemAt(2)->widget())
  //         ->setValue(leg[2].as<double>());
  //     gait_table_->setCellWidget(row, col, cell);
  //   }
  // }
}

#include <tuple> // for std::tie

// Helper: Convert HSV to RGB
std::tuple<float, float, float> hsvToRgb(float h, float s, float v) {
  float r, g, b;
  int i = int(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);
  switch (i % 6) {
  case 0:
    r = v;
    g = t;
    b = p;
    break;
  case 1:
    r = q;
    g = v;
    b = p;
    break;
  case 2:
    r = p;
    g = v;
    b = t;
    break;
  case 3:
    r = p;
    g = q;
    b = v;
    break;
  case 4:
    r = t;
    g = p;
    b = v;
    break;
  case 5:
    r = v;
    g = p;
    b = q;
    break;
  }
  return {r, g, b};
}

void HexapodControlRvizPanel::updateTableAndMarkers() {
  visualization_msgs::msg::MarkerArray marker_array;

  const int LEG_COUNT = 6;
  int POSE_COUNT = gait_table_->rowCount();
  int POINT_COUNT = POSE_COUNT * LEG_COUNT;

  for (int leg = 0; leg < LEG_COUNT; ++leg) {
    for (int row = 0; row < POSE_COUNT; ++row) {
      double x, y, z;
      PointInputLineEdit *widget =
          (PointInputLineEdit *)gait_table_->cellWidget(row, leg);
      widget->getPoint(x, y, z);

      // Rainbow color based on row index
      float hue = static_cast<float>(row) / std::max(1, POSE_COUNT - 1);
      float r, g, b;
      std::tie(r, g, b) = hsvToRgb(hue, 1.0, 1.0);

      // std::string positions[] = {"top_left", ""}
      // Sphere marker for point
      visualization_msgs::msg::Marker marker;
      // QString frame_id = positions.at(leg) + "_tibia";
      QString frame_id = positions.at(leg) + "_foot";
      marker.header.frame_id = frame_id.toStdString();
      // marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "hexapod_points";
      marker.id = row * LEG_COUNT + leg;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.scale.x = 0.015;
      marker.scale.y = 0.015;
      marker.scale.z = 0.015;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);

      // Add arrows from previous row to current, if this is not the first row
      if (row > 0) {
        double x_prev, y_prev, z_prev;
        PointInputLineEdit *prev_widget =
            (PointInputLineEdit *)gait_table_->cellWidget(row - 1, leg);
        prev_widget->getPoint(x_prev, y_prev, z_prev);

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id.toStdString();
        // arrow.header.stamp = rclcpp::Clock().now();
        arrow.ns = "hexapod_arrows";
        arrow.id =
            100000 + row * LEG_COUNT + leg; // Unique ID separate from spheres
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = x_prev;
        p_start.y = y_prev;
        p_start.z = z_prev;
        p_end.x = x;
        p_end.y = y;
        p_end.z = z;

        arrow.points.push_back(p_start);
        arrow.points.push_back(p_end);

        arrow.scale.x = 0.005; // shaft diameter
        arrow.scale.y = 0.02;  // head diameter
        arrow.scale.z = 0.03;  // head length

        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;

        marker_array.markers.push_back(arrow);
      }
    }
  }

  marker_array_pub_->publish(marker_array);
}
void HexapodControlRvizPanel::onAddRow() {
  visualization_msgs::msg::MarkerArray marker_array;
  int row = gait_table_->rowCount();
  gait_table_->insertRow(row);

  for (int leg = 0; leg < 6; ++leg) {
    PointInputLineEdit *point_widget = new PointInputLineEdit();
    gait_table_->setCellWidget(row, leg, point_widget);
  }
  updateTableAndMarkers();
}

void HexapodControlRvizPanel::onDeleteRow() {
  int row = gait_table_->currentRow();
  if (row >= 0)
    gait_table_->removeRow(row);
}

void HexapodControlRvizPanel::onMoveRowUp() {
  int row = gait_table_->currentRow();
  if (row > 0)
    swapRows(row, row - 1);
}

void HexapodControlRvizPanel::onMoveRowDown() {
  int row = gait_table_->currentRow();
  if (row < gait_table_->rowCount() - 1)
    swapRows(row, row + 1);
}

void HexapodControlRvizPanel::swapRows(int row1, int row2) {
  for (int col = 0; col < 6; ++col) {
    QWidget *w1 = gait_table_->cellWidget(row1, col);
    QWidget *w2 = gait_table_->cellWidget(row2, col);
    gait_table_->removeCellWidget(row1, col);
    gait_table_->removeCellWidget(row2, col);
    gait_table_->setCellWidget(row1, col, w2);
    gait_table_->setCellWidget(row2, col, w1);
  }
}
} // namespace hexapod_control_rviz_plugin

// Register the panel as a plugin
PLUGINLIB_EXPORT_CLASS(hexapod_control_rviz_plugin::HexapodControlRvizPanel,
                       rviz_common::Panel)

// PLUGINLIB_EXPORT_CLASS(hexapod_control_rviz_plugin::HexapodControlRvizPanel,
//                        rviz_common::Panel)
