#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include "hexapod_msgs/srv/solve_fk.hpp"
#include "hexapod_msgs/srv/solve_ik.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_control/hexapod.hpp>
#include <hexapod_control/requests.hpp>
#include <hexapod_control/ros_constants.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_pose.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <vector>

class PosePublisher : public rclcpp::Node {
protected:
  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Service<hexapod_msgs::srv::GetPose>::SharedPtr get_pose_service_;
  rclcpp::Service<hexapod_msgs::srv::SetPose>::SharedPtr set_pose_service_;
  rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedPtr solve_ik_client_;
  rclcpp::Client<hexapod_msgs::srv::SolveFK>::SharedPtr solve_fk_client_;
  rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_state_client_;

public:
  PosePublisher() : Node("leg_control_node") {
    using namespace std::chrono_literals;
    setupROS();
  }
  ~PosePublisher() {}

private:
  void setPose(const hexapod_msgs::msg::Pose pose) {
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<geometry_msgs::msg::Point> positions = pose.positions;
    solveIK(shared_from_this(), solve_ik_client_, pose.names, pose.positions,
            [this](const JointNames &joint_names,
                   const JointPositions &joint_positions) {
              setJointPositions(shared_from_this(), set_joint_state_client_,
                                joint_names, joint_positions);
            });

    // setJointPositions(shared_from_this(), set_joint_state_client_,
    // joint_names,
    //                   joint_positions);
  }

  void setupROS() {
    pose_pub_ = this->create_publisher<hexapod_msgs::msg::Pose>(
        POSE_TOPIC, rclcpp::QoS(10));

    solve_ik_client_ =
        create_client<hexapod_msgs::srv::SolveIK>(SOLVE_IK_SERVICE_NAME);

    solve_fk_client_ =
        create_client<hexapod_msgs::srv::SolveFK>(SOLVE_FK_SERVICE_NAME);

    set_joint_state_client_ = create_client<hexapod_msgs::srv::SetJointState>(
        SET_JOINT_STATE_SERVICE_NAME);

    get_pose_service_ = create_service<hexapod_msgs::srv::GetPose>(
        GET_POSE_SERVICE_NAME,
        [this](
            const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
            std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response) {
          getPose(shared_from_this(), request, response);
        });

    set_pose_service_ = create_service<hexapod_msgs::srv::SetPose>(
        SET_POSE_SERVICE_NAME,
        [this](
            const std::shared_ptr<hexapod_msgs::srv::SetPose::Request> request,
            std::shared_ptr<hexapod_msgs::srv::SetPose::Response> response) {
          setPose(request->pose);
          response->success = true;
          response->message = "Successfully set pose";
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
