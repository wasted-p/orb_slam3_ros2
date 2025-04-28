#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Global variables similar to your Python code
float rotation = 0.0;
std_msgs::msg::Float64MultiArray arm_joint_positions;

class JoystickReader : public rclcpp::Node {
public:
  JoystickReader() : Node("joystick_reader") {
    // Initialize arm joint positions
    arm_joint_positions.data = {0.0, 0.0, 0.0};
    
    // Create subscription to joy topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickReader::joy_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Joystick reader node initialized");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Extract joystick values from the message
    float mvmt_x = -msg->axes[0];  // Left joystick horizontal (inverted)
    float mvmt_y = msg->axes[1];   // Left joystick vertical
    
    float rtn_x = -msg->axes[2];   // Right joystick horizontal (inverted)
    float rtn_y = msg->axes[3];    // Right joystick vertical
    
    float step_x = msg->axes[4];   // D-pad or buttons (y axis)
    float step_y = msg->axes[5];   // D-pad or buttons (x axis)
    
    // Update global variables as in Python code
    arm_joint_positions.data[0] = rtn_x;
    arm_joint_positions.data[1] = rtn_y;
    arm_joint_positions.data[2] = 0.0;
    rotation = mvmt_y;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

class Commander : public rclcpp::Node {
public:
  Commander() : Node("commander"), counter_(0) {
    // Create publishers
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/legs_joint_trajectory_controller/joint_trajectory", 10);
    
    arm_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_joint_group_position_controller/commands", 10);
    
    // Set up timer
    double timer_period = 1.0; // seconds
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&Commander::publish_trajectory, this));
    
    // Initialize joint names
    joint_names_ = {
      "top_left_rotate_joint",
      "top_left_abduct_joint",
      "top_left_retract_joint",
      "mid_left_rotate_joint",
      "mid_left_abduct_joint",
      "mid_left_retract_joint",
      "bottom_left_rotate_joint",
      "bottom_left_abduct_joint",
      "bottom_left_retract_joint",
      "top_right_rotate_joint",
      "top_right_abduct_joint",
      "top_right_retract_joint",
      "mid_right_rotate_joint",
      "mid_right_abduct_joint",
      "mid_right_retract_joint",
      "bottom_right_rotate_joint",
      "bottom_right_abduct_joint",
      "bottom_right_retract_joint"
    };
    
    RCLCPP_INFO(this->get_logger(), "Commander node initialized");
  }

private:
  void publish_trajectory() {
    // Create trajectory message
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    
    // Set joint names for the trajectory
    trajectory_msg.joint_names = {
      "top_left_rotate_joint",
      "top_left_abduct_joint",
      "top_left_retract_joint",
      "bottom_left_rotate_joint",
      "bottom_left_abduct_joint",
      "bottom_left_retract_joint",
      "mid_right_rotate_joint",
      "mid_right_abduct_joint",
      "mid_right_retract_joint"
    };
    
    // Define trajectory points similar to Python code
    struct Point {
      std::string name;
      std::vector<double> positions;
      int duration;
    };
    
    std::vector<Point> points = {
      {
        "lift_legs",
        {0.0, -0.4, 0.0, 0.0, -0.4, 0.0, 0.0, -0.4, 0.0},
        1
      },
      {
        "lift_legs",
        {0.4, -0.4, 0.3, -0.4, -0.4, -0.5, -0.3, -0.4, 0.0},
        2
      }
    };
    
    // Add points to trajectory
    int t = 0;
    for (const auto& point : points) {
      auto p = trajectory_msgs::msg::JointTrajectoryPoint();
      p.positions = point.positions;
      t += point.duration;
      p.time_from_start.sec = point.duration;
      trajectory_msg.points.push_back(p);
    }
    
    // Publish trajectory
    publisher_->publish(trajectory_msg);
    
    // Modify and publish arm joint positions
    auto arm_positions = arm_joint_positions;
    arm_positions.data[0] = arm_positions.data[0] * M_PI * 0.5;
    arm_positions.data[1] = arm_positions.data[1] * M_PI * 0.5;
    // arm_positions.data[2] = arm_positions.data[2] * M_PI * 0.5;
    
    RCLCPP_INFO(this->get_logger(), "Published Joint Group Positions [%.2f, %.2f, %.2f]",
                arm_positions.data[0], arm_positions.data[1], arm_positions.data[2]);
    
    arm_publisher_->publish(arm_positions);
    
    counter_++;
  }
  
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  int counter_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  
  // Create multiple nodes
  auto joystick_reader = std::make_shared<JoystickReader>();
  auto commander = std::make_shared<Commander>();
  
  // Set up executor to handle multiple nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(joystick_reader);
  executor.add_node(commander);
  
  // Spin the executor in its own thread
  std::thread executor_thread([&executor]() { executor.spin(); });
  
  // Main thread can do other work or just wait
  executor_thread.join();
  
  rclcpp::shutdown();
  return 0;
}
