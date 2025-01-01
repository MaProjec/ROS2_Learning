import rclpy
from rclpy.node import Node


class ParamServer(Node):
    def __init__(self):
        super().__init__('param_server_py_node', allow_undeclared_parameters=False)
        self.get_logger().info('Parameter server node launched.')
    
    def declare_param(self):
        self.declare_parameter('car_name', 'tiger')
        self.declare_parameter('car_speed', 100)
        self.declare_parameter('car_weight', 1000.0)
    def get_param(self):
        car_name = self.get_parameter('car_name')
        car_speed = self.get_parameter('car_speed')
        car_weight = self.get_parameter('car_weight')
        self.get_logger().info('car_name: %s' % car_name.value)
        self.get_logger().info('car_speed: %d' % car_speed.value)
        self.get_logger().info('car_weight: %.2f' % car_weight.value)
        params = self.get_parameters(["car_name", "car_speed", "car_weight"])
        for param in params:
            self.get_logger().info('%s: %s' % (param.name, param.value))
        self.get_logger().info('has car_name? %d' % self.has_parameter('car_name'))
        self.get_logger().info('has car_color? %d' % self.has_parameter('car_color'))
    def update_param(self):
        # self.set_parameters([
        #     rclpy.Parameter('car_name',value = 'mouse')
        # ])
        car_name = self.get_parameter('car_name')
        self.get_logger().info('car_name: %s' % car_name.value)
    def del_param(self):
        self.get_logger().info('before delete has car_name? %d' % self.has_parameter('car_name'))
        #可以删除声明的参数
        self.undeclare_parameter('car_name')
        self.get_logger().info('after delete has car_name? %d' % self.has_parameter('car_name'))

def main(args=None):
    rclpy.init(args=args)
    node = ParamServer()
    node.declare_param()
    node.get_param()
    node.update_param()
    node.del_param()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()