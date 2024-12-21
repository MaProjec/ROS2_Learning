import rclpy
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts

class AddIntsServer(Node):

    def __init__(self):
        super().__init__('add_ints_server_node_py')
        self.get_logger().info('服务端创建了')
        self.srv = self.create_service(AddInts, 'add_ints', self.add_ints_callback)

    def add_ints_callback(self, request, response):
        response.sum = request.num1 + request.num2
        self.get_logger().info(f'{request.num1} + {request.num2} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()