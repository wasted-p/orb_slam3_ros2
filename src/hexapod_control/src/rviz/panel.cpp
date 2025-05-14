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

class PointInputWidget : public QWidget {
public:
  explicit PointInputWidget(QWidget *parent = nullptr) : QWidget(parent) {
    input_ = new QLineEdit(this);
    input_->setPlaceholderText("x,y,z");

    // Regex: allows patterns like -1.2,3,0 or 0,0.0,-5
    QRegExp rx(R"(^\s*-?\d+\.\d+\s*,\s*-?\d+\.\d+\s*,\s*-?\d+\.\d+\s*$)");
    QValidator *validator = new QRegExpValidator(rx, this);
    input_->setValidator(validator);

    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0); // clean fit into table
    layout->addWidget(input_);
    setLayout(layout);
    input_->setText("0.0, 0.0, 0.0");
    input_->setStyleSheet("QLineEdit { border: none; }");
    input_->setAlignment(Qt::AlignCenter);
    QLabel *left_bracket = new QLabel("(", this);
    QLabel *right_bracket = new QLabel(")", this);
    left_bracket->setStyleSheet("QLabel { font-weight: bold; }");
    right_bracket->setStyleSheet("QLabel { font-weight: bold; }");
    layout->addWidget(left_bracket);
    layout->addWidget(input_);
    layout->addWidget(right_bracket);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2); // Optional: adjust spacing between elements

    setLayout(layout);
  }

  QString text() const { return input_->text(); }
  void setText(const QString &text) { input_->setText(text); }
  bool getPoint(double &x, double &y, double &z) const {
    QStringList parts = input_->text().trimmed().split(',', Qt::SkipEmptyParts);
    if (parts.size() != 3) {
      return false;
    }

    bool ok1, ok2, ok3;
    x = parts[0].trimmed().toDouble(&ok1);
    y = parts[1].trimmed().toDouble(&ok2);
    z = parts[2].trimmed().toDouble(&ok3);

    return ok1 && ok2 && ok3;
  }

  bool setPoint(double x, double y, double z) {
    QString formatted = QString("%1, %2, %3")
                            .arg(x, 0, 'f', 2)
                            .arg(y, 0, 'f', 2)
                            .arg(z, 0, 'f', 2);
    input_->setText(formatted);
    return true;
  }

private:
  QLineEdit *input_;
};

HexapodControlRvizPanel::HexapodControlRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent), last_status_msg_("No status received yet") {
  // Set up the UI elements
  setupUi();

  // Create a timer for periodic updates
  update_timer_ = new QTimer(this);
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(updatePanel()));
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

  // Gait Editor Table
  gait_table_ = new QTableWidget(0, 6, this);
  QStringList headers;
  for (int i = 0; i < 6; ++i) {
    headers << QString("Leg %1").arg(i + 1);
    gait_table_->setColumnWidth(i, 100); // for 3 spin boxes side by side
  }
  gait_table_->setHorizontalHeaderLabels(headers);
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
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10);

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
    PointInputWidget *point_widget = new PointInputWidget();
    gait_table_->setCellWidget(row, leg, point_widget);
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

void HexapodControlRvizPanel::onAddRow() {
  int row = gait_table_->rowCount();
  gait_table_->insertRow(row);

  for (int leg = 0; leg < 6; ++leg) {
    PointInputWidget *point_widget = new PointInputWidget();
    gait_table_->setCellWidget(row, leg, point_widget);
  }
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
