from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class cmd_vel_selector(Node):

    def __init__(self):
        super().__init__('cmd_vel_selector')
        self.sub = self.create_subscription(Twist, '/cmd_vel_pad', self.cmd_vel_selector_callback,10)
        self.pub = self.create_publisher(Twist,'/cmd_vel_uart',10)

        self.vel = Twist() 
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0

    def cmd_vel_selector_callback(self, msg):
        self.vel.linear.x = msg.linear.x
        self.vel.linear.y = msg.linear.y

        self.vel.angular.z= msg.angular.z

        self.pub.publish(self.vel) 
        self.get_logger().info(f'Lx={self.vel.linear.x} Ly={self.vel.linear.y} Az={self.vel.angular.z}')


def main():
    rclpy.init()

    node = cmd_vel_selector()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
