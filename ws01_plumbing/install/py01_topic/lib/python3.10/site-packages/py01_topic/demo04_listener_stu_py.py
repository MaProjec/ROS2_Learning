import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

class ListenerStu(Node):

    def __init__(self):
        super().__init__('listenerstu_node_py')
        self.get_logger().info('订阅方创建了（python)')
        self.subscription = self.create_subscription(Student,'chatter_stu',self.listener_callback,10)

    def listener_callback(self, stu):
        self.get_logger().info('I heard: "%s,%d,%.2f"' % (stu.name , stu.age , stu.height))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ListenerStu())
    rclpy.shutdown()

if __name__ == '__main__':
    main()