import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ChallengeNode(Node):
    def __init__(self):
        super().__init__('challenge_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.execute()  #  lógica diretamente

    def move(self, linear, angular, duration):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start_time = self.get_clock().now().nanoseconds / 1e9  # Tempo inicial em segundos
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < duration:
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01) 
        # Para a tartaruga após o movimento
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)
        rclpy.spin_once(self, timeout_sec=0.01)

    def execute(self):
        for _ in range(4):  # 4 lados do quadrado
            self.get_logger().info('Movendo para frente...')
            self.move(linear=2.0, angular=0.0, duration=1.0)  # Anda rápido por 1 segundo
            
            self.get_logger().info('Girando 90 graus...')
            self.move(linear=0.0, angular=1.57, duration=1.0)  # Gira 90° aproximadamente (π/2 rad/s)
        
        self.get_logger().info('Quadrado completo!')

def main(args=None):
    rclpy.init(args=args)
    node = ChallengeNode()
    rclpy.spin(node)  # Mantém o nó 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
