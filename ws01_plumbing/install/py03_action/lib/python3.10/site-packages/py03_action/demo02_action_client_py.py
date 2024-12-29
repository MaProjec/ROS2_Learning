import rclpy
from rclpy.node import Node
import sys
from rclpy.action import ActionClient
from base_interfaces_demo.action import Progress
from rclpy.logging import get_logger

class ProgressActionClient(Node):
    def __init__(self):
        super().__init__('progress_action_client_node_py')
        self.get_logger().info("动作通信客户端创建")
        self.client = ActionClient(self, Progress, 'get_sum')
    
    def send_goal(self,num):
        #连接服务端
        self.client.wait_for_server()
        #发送请求
        goal = Progress.Goal()
        goal.num = num
        self.future = self.client.send_goal_async(goal,self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

    
    def feedback_callback(self,feedback_msg):
        progress = feedback_msg.feedback.progress
        self.get_logger().info("反馈进度: %.2f" % progress)

    def goal_response_callback(self,future):
        #获取目标句柄
        goal_handle = future.result()
        #通过打印获取类型
        #self.get_logger().info(goal_handle.__str__())
        #判断目标是否被正常接收
        if not goal_handle.accepted:
            self.get_logger().info("目标未被接收")
            return
        self.get_logger().info("目标已被接收")
        #处理最终响应结果
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self,future):
        result = future.result().result
        self.get_logger().info("最终结果: %d" % result.sum)

def main():
    if len(sys.argv) != 2:
        get_logger("r").error("清提交一个整型数据")
        return
    rclpy.init()
    action_client = ProgressActionClient()
    action_client.send_goal(int(sys.argv[1]))
    rclpy.spin(action_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
