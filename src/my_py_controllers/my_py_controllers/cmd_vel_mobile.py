import rclpy
import math

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class MyMobilleCmdVel(Node):
    def __init__(self):
        super().__init__("cmd_vel_mobile_controller")

        self.pose_ = None
        self.target_x = -5.0
        self.target_y = -5.0

        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber_ = self.create_subscription(
            Odometry, "/odom", self.callback_pose, 10
        )
        self.timer_ = self.create_timer(0.01, self.control_loop)

    def callback_pose(self, msg):
        self.pose_ = msg

    def control_loop(self):
        if self.pose_ == None:
            return
        # yaw (z-axis rotation)
        siny_cosp = 2 * (
            self.pose_.pose.pose.orientation.w * self.pose_.pose.pose.orientation.z
            + self.pose_.pose.pose.orientation.x * self.pose_.pose.pose.orientation.y
        )
        cosy_cosp = 1 - 2 * (
            self.pose_.pose.pose.orientation.y * self.pose_.pose.pose.orientation.y
            + self.pose_.pose.pose.orientation.z * self.pose_.pose.pose.orientation.z
        )

        yaw = math.atan2(siny_cosp, cosy_cosp)
        dist_x = self.target_x - self.pose_.pose.pose.position.x
        dist_y = self.target_y - self.pose_.pose.pose.position.y
        position_error = math.sqrt((math.pow(dist_x, 2)) + (math.pow(dist_y, 2)))

        msg = Twist()

        if position_error > 0.10:
            """Position"""
            msg.linear.x = 0.4

            """ Orientation """
            desired_theta = math.atan2(dist_y, dist_x)
            orientation_error = desired_theta - yaw

            """ Bound error """
            orientation_error = math.atan2(
                math.sin(orientation_error), math.cos(orientation_error)
            )

            msg.angular.z = (
                1 * orientation_error - 0.5 * self.pose_.twist.twist.angular.z
            )

        elif position_error <= 0.10 and position_error > 0.025:
            """Position"""
            msg.linear.x = 0.3 * position_error
            msg.angular.z = 0.3 * position_error
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyMobilleCmdVel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
