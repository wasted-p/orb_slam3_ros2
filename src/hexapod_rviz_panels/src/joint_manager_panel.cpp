#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <hexapod_rviz_panels/joint_manager_panel.hpp>

namespace hexapod_rviz_panels
{
JointManagerPanel::JointManagerPanel(QWidget * parent) : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("[no data]");
  button_ = new QPushButton("GO!");
  layout->addWidget(label_);
  layout->addWidget(button_);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the callback being called.
  QObject::connect(button_, &QPushButton::released, this, &JointManagerPanel::buttonActivated);
}

JointManagerPanel::~JointManagerPanel() = default;

void JointManagerPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_ = node->create_publisher<std_msgs::msg::String>("/output", 10);
  subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/input", 10, std::bind(&JointManagerPanel::topicCallback, this, std::placeholders::_1));
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void JointManagerPanel::topicCallback(const std_msgs::msg::String& msg)
{
  label_->setText(QString(msg.data.c_str()));
}

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void JointManagerPanel::buttonActivated()
{
  auto message = std_msgs::msg::String();
  message.data = "Button clicked!";
  publisher_->publish(message);
}

}  // namespace hexapod_rviz_panels

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hexapod_rviz_panels::JointManagerPanel, rviz_common::Panel)
