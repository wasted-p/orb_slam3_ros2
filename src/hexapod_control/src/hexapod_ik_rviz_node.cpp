#include <hexapod_control/hexapod_ik_base.hpp>

class HexapodIKRvizNode : public HexapodIKBaseNode {
  void updatePose(const hexapod_msgs::msg::Pose pose) {
    std::vector<double> joint_positions;
    std::string leg_name;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Pose int_marker_pose;

    joint_state_msg_.name = {};
    joint_state_msg_.position = {};

    for (size_t i = 0; i < pose.names.size(); i++) {
      {
        leg_name = pose.names[i];
        position = pose.positions[i];
        RCLCPP_DEBUG(
            get_logger(),
            "Recieved Leg Pose Message for leg %s with positions=[%.2f, "
            "%.2f, %.2f]",
            leg_name.c_str(), position.x, position.y, position.z);

        int status = planning_group.calculateJntArray(
            chains_.at(leg_name), position, joint_positions);

        if (status < 0) {
          RCLCPP_ERROR(get_logger(), "Error %i", status);
          return;
        }
        joint_state_msg_.header.frame_id = "base_footprint";

        joint_state_msg_.name.insert(joint_state_msg_.name.cend(),
                                     {leg_name + "_rotate_joint",
                                      leg_name + "_abduct_joint",
                                      leg_name + "_retract_joint"});

        joint_state_msg_.position.insert(
            joint_state_msg_.position.cend(),
            {joint_positions[0], joint_positions[1], joint_positions[2]});

        RCLCPP_DEBUG(
            get_logger(), "Sending Target Joint Positions = [%.2f, %.2f, %.2f]",
            joint_positions[0], joint_positions[1], joint_positions[2]);

        int_marker_pose.position = position;
        server_->setPose(leg_name, int_marker_pose);
        server_->applyChanges();
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HexapodIKRvizNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
