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

        timer_period = 1
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
        trajectory_msg.joint_names = [
            "top_left_rotate_joint",
            "top_left_abduct_joint",
            "top_left_retract_joint",

            "bottom_left_rotate_joint",
            "bottom_left_abduct_joint",
            "bottom_left_retract_joint",

            "mid_right_rotate_joint",
            "mid_right_abduct_joint",
            "mid_right_retract_joint",

        ]
        points = [
            {
                "name":"lift_legs",
                "positions" : [0.0,-0.4,0.0,
                               0.0,-0.4,0.0,
                               0.0,-0.4,0.0],
                "duration" :1,
            },

            {
                "name":"lift_legs",
                "positions" : [0.4,-0.4,0.3,
                               -0.4,-0.4,-0.5,
                               -0.3,-0.4,0],
                "duration" :2,
            }
        ]

        t = 0
        for point in points:
            p = JointTrajectoryPoint()
            p.positions = point["positions"]
            t += point["duration"]
            p.time_from_start.sec = point["duration"]
            trajectory_msg.points.append(p)

        self.publisher.publish(trajectory_msg)

        # publishing joint positions for arm
        arm_joint_positions.data[0] = arm_joint_positions.data[0] * math.pi * 0.5
        arm_joint_positions.data[1] = arm_joint_positions.data[1] * math.pi * 0.5
        # arm_joint_positions.data[2] = arm_joint_positions.data[2] * math.pi * 0.5

        self.get_logger().info(f'Published Joint Group Positions {arm_joint_positions.data}')
        self.arm_publisher.publish(arm_joint_positions)

        self.counter += 1


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

