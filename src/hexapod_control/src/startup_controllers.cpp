
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <hexapod_common/yaml_utils.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

std::vector<std::string>
extractControllersFromYaml(const std::string &config_path,
                           const std::string &prefix) {
  std::vector<std::string> controllers;

  try {
    YAML::Node config = YAML::LoadFile(config_path);

    // Direct access to hexapod namespace
    // if (config[prefix]) {
    for (const auto &node : config[prefix]) {
      std::string node_name = node.first.as<std::string>();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("TEST"), node_name);

      // Skip controller_manager as it's not a controller itself
      if (node_name != "controller_manager") {
        controllers.push_back(node_name);
      }
    }

  } catch (const YAML::Exception &e) {
    std::cerr << "Error parsing YAML: " << e.what() << std::endl;
  }

  return controllers;
}
using namespace std::chrono_literals;

class StartupControllersNode : public rclcpp::Node {

private:
  std::string prefix_;

public:
  ~StartupControllersNode() {}

  StartupControllersNode() : Node("startup_controllers_node") {
    // load_client_ =
    // create_client<controller_manager_msgs::srv::LoadController>(
    //     joinWithSlash(prefix_, "controller_manager/load_controller"));
    // unload_client_ =
    //     create_client<controller_manager_msgs::srv::UnloadController>(
    //         joinWithSlash(prefix_, "controller_manager/unload_controller"));
    // configure_client_ =
    //     create_client<controller_manager_msgs::srv::ConfigureController>(
    //         joinWithSlash(prefix_,
    //         "controller_manager/configure_controller"));
    // switch_client_ =
    //     this->create_client<controller_manager_msgs::srv::SwitchController>(
    //         joinWithSlash(prefix_, "controller_manager/switch_controller"));

    loadParams();
    setupControllerManager();
    wait_for_services();
    setup_controllers();
  }
  void loadParams() {
    prefix_ = this->declare_parameter("prefix", std::string(""));
    RCLCPP_INFO(get_logger(), "Prefix: %s", prefix_.c_str());
    std::string config_path =
        this->declare_parameter("config_path", std::string());

    RCLCPP_INFO(get_logger(), "Reading config from: %s", config_path.c_str());
    RCLCPP_INFO(get_logger(), "Found following controllers:");
    controllers_ = extractControllersFromYaml(config_path, prefix_);
    for (const std::string &name : controllers_) {
      RCLCPP_INFO_STREAM(get_logger(), " - " << name);
    }
  }

  void setupControllerManager() {
    load_client_ = create_client<controller_manager_msgs::srv::LoadController>(
        joinWithSlash(
            prefix_,
            "controller_manager/load_controller")); // Add namespace here
    unload_client_ =
        create_client<controller_manager_msgs::srv::UnloadController>(
            joinWithSlash(
                prefix_,
                "controller_manager/unload_controller")); // Add namespace
                                                          // here
    configure_client_ =
        create_client<controller_manager_msgs::srv::ConfigureController>(
            joinWithSlash(
                prefix_,
                "controller_manager/configure_controller")); // Add
                                                             // namespace
                                                             // here
    switch_client_ =
        this->create_client<controller_manager_msgs::srv::SwitchController>(
            joinWithSlash(
                prefix_,
                "controller_manager/switch_controller")); // Add namespace
                                                          // here
  }

  void wait_for_services() {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for controller_manager services...");
    while (!load_client_->wait_for_service(1s) ||
           !configure_client_->wait_for_service(1s) ||
           !switch_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting...");
    }
    RCLCPP_INFO(this->get_logger(), "All services available.");
  }

  void loadController(std::string name) {

    // Load
    auto load_req = std::make_shared<
        controller_manager_msgs::srv::LoadController::Request>();
    load_req->name = name;
    auto load_result = load_client_->async_send_request(load_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           load_result) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !load_result.get()->ok) {
      throw std::runtime_error("Failed to load controller:" + name);
    }
  }
  void configureController(std::string name) {
    auto config_req = std::make_shared<
        controller_manager_msgs::srv::ConfigureController::Request>();
    config_req->name = name;
    auto config_result = configure_client_->async_send_request(config_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           config_result) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !config_result.get()->ok) {
      throw std::runtime_error("Failed to configure controller:" + name);
    }
  }
  void setup_controllers() {
    for (const auto &name : controllers_) {
      try {
        loadController(name);
        RCLCPP_INFO(this->get_logger(), "Configured controller: %s",
                    name.c_str());
        configureController(name);
        RCLCPP_INFO(this->get_logger(), "Loaded controller: %s", name.c_str());
      } catch (const std::runtime_error e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
      }
    }
    auto switch_req = std::make_shared<
        controller_manager_msgs::srv::SwitchController::Request>();
    switch_req->activate_controllers = controllers_;
    switch_req->deactivate_controllers = {};
    switch_req->strictness =
        controller_manager_msgs::srv::SwitchController::Request::STRICT;

    auto switch_result = switch_client_->async_send_request(switch_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           switch_result) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !switch_result.get()->ok) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start controllers");
    } else {
      RCLCPP_INFO(this->get_logger(), "Started controllers successfully");
    }
  }

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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Create node and immediately run task
  {
    auto node = std::make_shared<StartupControllersNode>();
    // No spin — the node will be destructed at end of scope
  }
  rclcpp::shutdown();
  return 0;
}
