
#ifndef ROS_CONSTANTS_HPP
#define ROS_CONSTANTS_HPP

#define POSE_TOPIC "pose"
#define JOINT_STATE_TOPIC "joint_states"
#define MARKER_ARRAY_TOPIC "visualization/leg_pose_markers"
#define SET_POSE_SERVICE_NAME "pose/set"
#define GET_POSE_SERVICE_NAME "pose/get"
#define SOLVE_IK_SERVICE_NAME "solve_ik"
#define SOLVE_FK_SERVICE_NAME "solve_fk"
#define SET_JOINT_STATE_SERVICE_NAME "joint_states/set"
#define INTERACTIVE_MARKERS_SERVER_NAME "interactive_marker_server"
#define SET_MARKER_ARRAY_SERVICE_NAME "set_marker_array"
#define EXECUTE_MOTION_SERVICE_NAME "motion_server/motion/execute"
#define JOY_TOPIC "joy"

#define TRAJECTORY_SERVICE_NAME                                                \
  "legs_joint_trajectory_controller/follow_joint_trajectory"
#endif // !ROS_CONSTANTS_HPP
