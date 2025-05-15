// hexapod_control_rviz_panel.cpp
#include "rviz/panel.hpp"
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>

namespace hexapod_control_rviz_plugin {

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

  // Text input field
  text_input_ = new QLineEdit();
  text_input_->setPlaceholderText("Enter command text");
  main_layout_->addWidget(text_input_);

  // Dropdown menu
  option_combo_ = new QComboBox();
  option_combo_->addItem("Option 1");
  option_combo_->addItem("Option 2");
  option_combo_->addItem("Option 3");
  main_layout_->addWidget(option_combo_);

  // Buttons layout
  buttons_layout_ = new QHBoxLayout;
  main_layout_->addLayout(buttons_layout_);

  // Control buttons
  button1_ = new QPushButton("Execute");
  connect(button1_, SIGNAL(clicked()), this, SLOT(onButton1Clicked()));
  buttons_layout_->addWidget(button1_);

  button2_ = new QPushButton("Reset");
  connect(button2_, SIGNAL(clicked()), this, SLOT(onButton2Clicked()));
  buttons_layout_->addWidget(button2_);
}

void HexapodControlRvizPanel::setupROS() {
  // Create node with unique name
  node_ = std::make_shared<rclcpp::Node>("hexapod_control_rviz_panel_node");

  // Create publishers
  pub_command_ =
      node_->create_publisher<std_msgs::msg::String>("panel_commands", 10);

  // Set up subscriptions
  subscribeToTopics();

  // Spin the node in a separate thread
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_);
  std::thread([executor]() { executor->spin(); }).detach();
}

void HexapodControlRvizPanel::subscribeToTopics() {
  // Subscribe to status updates
  sub_status_ = node_->create_subscription<std_msgs::msg::String>(
      "status_updates", 10,
      std::bind(&HexapodControlRvizPanel::statusCallback, this,
                std::placeholders::_1));
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

void HexapodControlRvizPanel::onButton1Clicked() {
  // Get the command text from the input field
  std::string command = text_input_->text().toStdString();
  if (!command.empty()) {
    publishCommand(command);
    text_input_->clear();
  }
}

void HexapodControlRvizPanel::onButton2Clicked() {
  // Reset the panel state
  text_input_->clear();
  option_combo_->setCurrentIndex(0);
  publishCommand("reset");
}

void HexapodControlRvizPanel::publishCommand(const std::string &command) {
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = command;
  pub_command_->publish(std::move(msg));
}

void HexapodControlRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);

  // Save UI state to config
  config.mapSetValue("text", text_input_->text().toStdString().c_str());
  config.mapSetValue("option", option_combo_->currentIndex());
}

void HexapodControlRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);

  // Load UI state from config
  QString text;
  if (config.mapGetString("text", &text)) {
    text_input_->setText(text);
  }

  int option_index;
  if (config.mapGetInt("option", &option_index)) {
    option_combo_->setCurrentIndex(option_index);
  }
}

} // namespace hexapod_control_rviz_plugin

// Register the panel as a plugin
PLUGINLIB_EXPORT_CLASS(hexapod_control_rviz_plugin::HexapodControlRvizPanel,
                       rviz_common::Panel)
