#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

class ShutdownControllersNode : public rclcpp::Node {
private:
  std::vector<std::string> controllers_;
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr
      load_client_;
  rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr
      unload_client_;
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr
      configure_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_client_;
  rclcpp::TimerBase::SharedPtr wait_timer_;

public:
  ShutdownControllersNode() : Node("shutdown_controllers_node") {

    load_client_ = create_client<controller_manager_msgs::srv::LoadController>(
        "/controller_manager/load_controller");

    unload_client_ =
        create_client<controller_manager_msgs::srv::UnloadController>(
            "/controller_manager/unload_controller");
    configure_client_ =
        create_client<controller_manager_msgs::srv::ConfigureController>(
            "/controller_manager/configure_controller");
    switch_client_ =
        this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

    controllers_ = {"joint_state_broadcaster",
                    "legs_joint_trajectory_controller",
                    "arm_joint_group_position_controller"};

    shutdownControllers();
  }
  void shutdownControllers() {
    RCLCPP_INFO(get_logger(), "Shutting down controllers");
    // Deactivate
    auto switch_req = std::make_shared<
        controller_manager_msgs::srv::SwitchController::Request>();
    switch_req->deactivate_controllers = controllers_;
    switch_req->strictness =
        controller_manager_msgs::srv::SwitchController::Request::STRICT;

    auto switch_result = switch_client_->async_send_request(switch_req);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                       switch_result);

    // Unload
    for (const auto &controller : controllers_) {
      auto unload_req = std::make_shared<
          controller_manager_msgs::srv::UnloadController::Request>();
      unload_req->name = controller;
      auto unload_result = unload_client_->async_send_request(unload_req);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         unload_result);

      RCLCPP_INFO(get_logger(), "Unloaded controller: %s", controller.c_str());
    }
  }
  ~ShutdownControllersNode() {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Create node and immediately run task
  {
    auto node = std::make_shared<ShutdownControllersNode>();
    // No spin — the node will be destructed at end of scope
  }
  rclcpp::shutdown();
  return 0;
}
