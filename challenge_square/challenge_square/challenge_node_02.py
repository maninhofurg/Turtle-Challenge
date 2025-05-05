import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ChallengeNode(Node):
    def __init__(self):
        super().__init__('challenge_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, lambda: None)
        time.sleep(2)  # Aguarda tudo iniciar
        self.execute()

    def send_twist(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        start_time = self.get_clock().now().nanoseconds / 1e9
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < duration:
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop()

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)
        rclpy.spin_once(self, timeout_sec=0.05)

    def execute(self):
        for i in range(4):
            self.get_logger().info(f' Lado {i+1}')
            self.send_twist(linear=1.0, angular=0.0, duration=2.0)   # vai reto
            self.stop()
            time.sleep(0.3)
            self.send_twist(linear=0.0, angular=1.57, duration=1.0)  # ele vai gira 90°
            self.stop()
            time.sleep(0.3)

def main(args=None):
    rclpy.init(args=args)
    node = ChallengeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

