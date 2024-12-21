import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener_node_py')
        self.get_logger().info('订阅方创建了（python)')
        self.subscription1 = self.create_subscription(String, "chatter", self.do_cb, 10)
    def do_cb(self, message):
        self.get_logger().info('订阅消息：%s' % message.data)


def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()

if __name__ == '__main__':
    main()