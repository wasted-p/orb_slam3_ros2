import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')

        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

        self.joint_names = ['arm_rotator_joint']

        self.counter = 0
    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        point.positions = [
            0.5
        ]

        point.velocities = [0.0]
        point.time_from_start.sec = 1

        trajectory_msg.points.append(point)

        self.publisher.publish(trajectory_msg)
        self.get_logger().info(f'Published trajectory {trajectory_msg}')

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
