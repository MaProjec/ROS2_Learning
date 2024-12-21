import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student


class TalkeStu(Node):
    def __init__(self):
        super().__init__('talkerstu_node_py')
        self.publisher_ = self.create_publisher(Student, 'chatter_stu', 10)
        self.create_timer(0.5, self.timer_callback)
        self.age = 0

    def timer_callback(self):
        stu = Student()
        stu.name = "张三"
        stu.age = self.age
        stu.height = 1.65
        self.publisher_.publish(stu)
        self.get_logger().info(f'Publishing: "{stu.name},{stu.age},{stu.height}"')
        self.age += 1

def main(args=None):
    rclpy.init()
    rclpy.spin(TalkeStu())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
