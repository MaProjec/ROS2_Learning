import rclpy
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts
import sys
from rclpy.logging import get_logger

class AddIntsClient(Node):
    def __init__(self):
        super().__init__('add_ints_client_node_py')
        self.get_logger().info('客户端创建了')
        self.client_ = self.create_client(AddInts, 'add_ints')
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info('服务端没有启动，等待中...')
    def send_request(self):
        request = AddInts.Request()
        request.num1 = int(sys.argv[1])
        request.num2 = int(sys.argv[2])
        self.future = self.client_.call_async(request)  #响应，是一个future对象
        

    
def main():
    if len(sys.argv) < 3:
        #rclpy.logging.get_logger("rclpy").error("请输入两个整数")
        get_logger("rclpy").error("请输入两个整数")
        return
    rclpy.init()
    client = AddIntsClient()
    client.send_request()
    rclpy.spin_until_future_complete(client, client.future)
    try:
        response = client.future.result()
        client.get_logger().info(f'{sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    except Exception as e:
        client.get_logger().error(f'Service call failed {e}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()