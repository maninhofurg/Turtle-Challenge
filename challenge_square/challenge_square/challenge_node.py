import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ChallengeNode(Node):
    def __init__(self):
        super().__init__('challenge_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.execute()

    def move(self, linear, angular, duration):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        clock = self.get_clock()
        start_time = clock.now()
        elapsed = rclpy.duration.Duration(seconds=0)

        while elapsed < rclpy.duration.Duration(seconds=duration):
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            elapsed = clock.now() - start_time

        # Para a tartaruga após o movimento
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)
        rclpy.spin_once(self, timeout_sec=0.01)

    def execute(self):
        for _ in range(4):  # 4 lados do quadrado
            self.get_logger().info('Movendo para frente...')
            self.move(linear=2.0, angular=0.0, duration=1.0)
            
            self.get_logger().info('Girando 90 graus...')
            self.move(linear=0.0, angular=1.57, duration=1.0)

        self.get_logger().info('Quadrado completo!')

def main(args=None):
    rclpy.init(args=args)
    node = ChallengeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

