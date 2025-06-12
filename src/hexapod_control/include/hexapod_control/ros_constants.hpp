#ifndef ROS_CONSTANTS_HPP
#define ROS_CONSTANTS_HPP

#define POSE_TOPIC "/hexapod/pose"
#define JOINT_STATE_TOPIC "/joint_states"
#define SET_POSE_SERVICE_NAME "/hexapod/pose/set"
#define GET_POSE_SERVICE_NAME "/hexapod/pose/get"
#define SOLVE_IK_SERVICE_NAME "/kinematics/solve_ik"
#define SOLVE_FK_SERVICE_NAME "/kinematics/solve_fk"
#define SET_JOINT_STATE_SERVICE_NAME "/joint_state/set"
#define INTERACTIVE_MARKERS_SERVER_NAME "interactive_marker_server"
#define SET_MARKER_ARRAY_SERVICE_NAME "/set_marker_array"

#endif // !ROS_CONSTANTS_HPP
