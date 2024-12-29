import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import rclpy.action
from base_interfaces_demo.action import Progress
import time

class ProgressActionServer(Node):
    def __init__(self):
        super().__init__('progress_action_server_node_py')
        self.get_logger().info("动作通信服务端创建")
        self.server = ActionServer(self, Progress, 'get_sum', self.execute_callback,
                                   goal_callback = self.goal_accept_callback,
                                   cancel_callback=self.cancel_callback)

    def execute_callback(self,goal_handle):
        num = goal_handle.request.num
        sum = 0
        for i in range(1,num+1):
            sum += i
            if goal_handle.is_cancel_requested:
                self.get_logger().info("目标已取消")
                goal_handle.canceled()
                result = Progress.Result()
                result.sum = sum
                return result            
            #创建反馈对象
            feedback = Progress.Feedback()
            feedback.progress = i / num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("反馈进度: %.2f" % feedback.progress)
            #模拟耗时操作
            time.sleep(1)
        #标记一个目标（goal_handle）已经成功完成
        goal_handle.succeed()
        #创建结果对象
        result = Progress.Result()
        result.sum = sum
        self.get_logger().info("返回结果: %d" % sum)
        return result
    
    #重写目标处理(接受)函数
    def goal_accept_callback(self, goal_request):
        if goal_request.num <= 1 :
            self.get_logger().info("目标<=1，无法接受")
            return rclpy.action.GoalResponse.REJECT
        else:
            self.get_logger().info("目标已接受")
            return rclpy.action.GoalResponse.ACCEPT
    
    #重写取消处理(接受)函数
    def cancel_callback(self,cancel_request):
        self.get_logger().info("目标已取消")
        return rclpy.action.CancelResponse.ACCEPT
    
def main():
    rclpy.init()
    action_server = ProgressActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()