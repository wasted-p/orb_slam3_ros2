#include "hexapod_control/hexapod_control_rviz_panel.hpp"

namespace hexapod_control_rviz_plugin {

HexapodControlRvizPanel::HexapodControlRvizPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  // Set up the UI elements
  setupUi();
}

HexapodControlRvizPanel::~HexapodControlRvizPanel() {}

void HexapodControlRvizPanel::onInitialize() {
  // Create ROS node and set up ROS communications
  setupROS();
}

void HexapodControlRvizPanel::setupUi() {
  PosesTab *poses_tab = new PosesTab;
  CommandsTab *commands_tab = new CommandsTab;
  WarehouseTab *warehouse_tab = new WarehouseTab;

  // Main layout
  main_layout_ = new QVBoxLayout;
  setLayout(main_layout_);

  // Main Tab Widget for Warehouse, MotionPlanning, and Views
  main_tab_widget_ = new QTabWidget;
  main_layout_->addWidget(main_tab_widget_);

  main_tab_widget_->addTab(commands_tab, "Commands");

  // Placeholder for other MotionPlanning sub-tabs
  main_tab_widget_->addTab(poses_tab, "Poses");

  main_tab_widget_->addTab(warehouse_tab, "Warehouse");

  main_tab_widget_->addTab(new QWidget, "Cycles");

  // Add the tab to the main tab widget
}

void HexapodControlRvizPanel::setupROS() {
  node_ = NodeManager::getNode();
  pub_command_ =
      node_->create_publisher<std_msgs::msg::String>("panel_commands", 10);
  subscribeToTopics();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_);
  std::thread([executor]() { executor->spin(); }).detach();
}

void HexapodControlRvizPanel::subscribeToTopics() {
  // sub_status_ = node_->create_subscription<std_msgs::msg::String>(
  //     "status_updates", 10,
  //     std::bind(&HexapodControlRvizPanel::statusCallback, this,
  //               std::placeholders::_1));
}

void HexapodControlRvizPanel::updatePanel() {
  // Placeholder for panel updates
}

void HexapodControlRvizPanel::save(rviz_common::Config config) const {
  Panel::save(config);
}

void HexapodControlRvizPanel::load(const rviz_common::Config &config) {
  Panel::load(config);
}

} // namespace hexapod_control_rviz_plugin

PLUGINLIB_EXPORT_CLASS(hexapod_control_rviz_plugin::HexapodControlRvizPanel,
                       rviz_common::Panel)
