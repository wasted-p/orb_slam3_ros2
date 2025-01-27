#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

# class Commander(Node):
#
#     def __init__(self):
#         super().__init__('commander')
#         timer_period = 0.02
#         self.wheel_seperation = 0.122
#         self.wheel_base = 0.156
#         self.wheel_radius = 0.026
#         self.wheel_steering_y_offset = 0.03
#         self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset
#
#         self.pos = np.array([0,0,0,0], float)
#         self.vel = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear
#
#         self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
#         self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#
#     def timer_callback(self):
#         global vel_msg, mode_selection
#
#         # opposite phase
#         if(mode_selection == 1):
#
#             vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
#             sign = np.sign(vel_msg.linear.x)
#
#             self.vel[0] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
#             self.vel[1] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset
#             self.vel[2] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
#             self.vel[3] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset
#
#             a0 = 2*vel_msg.linear.x + vel_msg.angular.z*self.steering_track
#             a1 = 2*vel_msg.linear.x - vel_msg.angular.z*self.steering_track
#
#             if a0 != 0:
#                 self.pos[0] = math.atan(vel_msg.angular.z*self.wheel_base/(a0))
#             else:
#                 self.pos[0] = 0
#
#             if a1 != 0:
#                 self.pos[1] = math.atan(vel_msg.angular.z*self.wheel_base/(a1))
#             else:
#                 self.pos[1] = 0
#
#             self.pos[2] = -self.pos[0]
#             self.pos[3] = -self.pos[1]
#
#         # in-phase
#         elif(mode_selection == 2):
#
#             V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
#             sign = np.sign(vel_msg.linear.x)
#
#             if(vel_msg.linear.x != 0):
#                 ang = vel_msg.linear.y / vel_msg.linear.x
#             else:
#                 ang = 0
#
#             self.pos[0] = math.atan(ang)
#             self.pos[1] = math.atan(ang)
#             self.pos[2] = self.pos[0]
#             self.pos[3] = self.pos[1]
#
#             self.vel[:] = sign*V
#
#         # pivot turn
#         elif(mode_selection == 3):
#
#             self.pos[0] = -math.atan(self.wheel_base/self.steering_track)
#             self.pos[1] = math.atan(self.wheel_base/self.steering_track)
#             self.pos[2] = math.atan(self.wheel_base/self.steering_track)
#             self.pos[3] = -math.atan(self.wheel_base/self.steering_track)
#
#             self.vel[0] = -vel_msg.angular.z
#             self.vel[1] = vel_msg.angular.z
#             self.vel[2] = self.vel[0]
#             self.vel[3] = self.vel[1]
#
#         else:
#
#             self.pos[:] = 0
#             self.vel[:] = 0
#
#         pos_array = Float64MultiArray(data=self.pos) 
#         vel_array = Float64MultiArray(data=self.vel) 
#         self.pub_pos.publish(pos_array)
#         self.pub_vel.publish(vel_array)
#         self.pos[:] = 0
#         self.vel[:] = 0

rotation = 0
arm_joint_positions = Float64MultiArray()
arm_joint_positions.data = [0,0,0]

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global rotation, arm_joint_positions

        mvmt_x = -data.axes[0] # Correlates to left joystick
        mvmt_y = data.axes[1] # Correlates to left joystick

        rtn_x = -data.axes[2] # Correlates to right joystick
        rtn_y = data.axes[3] # Correlates to right joystick

        step_x = data.axes[4] # Correlates to buttons (y axis)
        step_y = data.axes[5] # Correlates to right (x axis)

        arm_joint_positions.data = [rtn_x, rtn_y, 0]
        rotation = mvmt_y


class Commander(Node):
    def __init__(self):
        super().__init__('commander')

        self.publisher = self.create_publisher(JointTrajectory, '/legs_joint_trajectory_controller/joint_trajectory', 10)
        self.arm_publisher = self.create_publisher(Float64MultiArray, '/arm_joint_group_position_controller/commands', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

        self.joint_names = [
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
            "bottom_right_retract_joint",
        ]

        self.counter = 0

    def publish_trajectory(self):
        global arm_joint_positions, rotation
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ["top_left_abduct_joint", "bottom_left_abduct_joint", "mid_right_abduct_joint"]

        point = JointTrajectoryPoint()

        point.positions = [
            rotation, rotation, rotation
        ]

        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 1

        trajectory_msg.points.append(point)
        self.publisher.publish(trajectory_msg)

        # publishing joint positions for arm
        arm_joint_positions.data[0] = arm_joint_positions.data[0] * math.pi * 0.5
        arm_joint_positions.data[1] = arm_joint_positions.data[1] * math.pi * 0.5
        # arm_joint_positions.data[2] = arm_joint_positions.data[2] * math.pi * 0.5

        # self.get_logger().info(f'Published Joint Group Positions {arm_joint_positions.data}')
        self.arm_publisher.publish(arm_joint_positions)

        self.counter += 1

# def main(args=None):
#     rclpy.init(args=args)
#     node = JointTrajectoryPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
#

if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

