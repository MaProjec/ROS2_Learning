import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker_node_py')
        self.get_logger().info('发布方创建了（python)')
        self.count = 0
        self.publisher1 = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(1.0, self.on_time)
    def on_time(self):
        msg = String()
        msg.data = "hello world " + str(self.count) 
        self.publisher1.publish(msg)
        self.count += 1
        self.get_logger().info('发布消息：%s' % msg.data)

def main():
    rclpy.init()
    rclpy.spin(Talker())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
