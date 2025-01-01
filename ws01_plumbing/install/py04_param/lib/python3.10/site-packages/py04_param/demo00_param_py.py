import rclpy
from rclpy.node import Node

class MyParam(Node):
    def __init__(self):
        super().__init__('my_param_node_py')
        self.get_logger().info('myparam node created')

        #创建参数对象
        p1 = rclpy.Parameter('car_name', value ="tiger")
        p2 = rclpy.Parameter('car_speed', value = 100)
        p3 = rclpy.Parameter('car_weight', value = 1000.0)
        #解析参数值
        self.get_logger().info('car_name: %s' % p1.value)
        self.get_logger().info('car_speed: %d' % p2.value)
        self.get_logger().info('car_weight: %.2f' % p3.value)

        self.get_logger().info('name(键) = %s'% p1.name)

def main(args=None):
    rclpy.init(args=args)
    node = MyParam()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
