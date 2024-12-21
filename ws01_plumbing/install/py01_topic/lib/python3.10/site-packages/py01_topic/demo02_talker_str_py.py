import rclpy
from rclpy.node import Node

class Listener(Node):
    def __init__(self):
        super.__init__('listener_node_py')
        self.get_logger().info('订阅方创建了（python)')
        

def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()

if __name__ == '__main__':
    main()