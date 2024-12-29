#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;
class ProgressActionServer : public rclcpp::Node
{
public:
    ProgressActionServer():Node("progress_action_server_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "action server constructed");
        server_ = rclcpp_action::create_server<Progress>(
            this,
            "get_sum",
            std::bind(&ProgressActionServer::handle_goal, this, _1, _2),
            std::bind(&ProgressActionServer::handle_cancel, this, _1),
            std::bind(&ProgressActionServer::handle_accepted, this, _1)
        );
    }
/*std::function<GoalResponse(
        const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;*/
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const Progress::Goal> goal)
    {
        (void)uuid;     // 未使用的参数，防止编译器警告
        if(goal->num <= 1)
        {
            RCLCPP_INFO(this->get_logger(), "提交的目标值小于等于1，拒绝执行");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "接受目标值：%d", goal->num);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

/*CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;*/
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
        //为接收到取消请求的处理
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "开始执行");
        int num = goal_handle->get_goal()->num;
        int sum = 0;
        auto feedback = std::make_shared<Progress::Feedback>();
        auto result = std::make_shared<Progress::Result>();
        //设置休眠
        rclcpp::Rate rate(1);
        for(int i = 1; i <= num; i++)
        {
            sum += i;
            //检查是否有取消请求
            if(goal_handle->is_canceling())
            {
                RCLCPP_INFO(this->get_logger(), "任务已取消");      
                result->sum = sum;
                // 生成最终响应结果
                goal_handle->canceled(result);
                return;
            }
            double progress = i / (double)num;
            feedback->progress = progress;
            //生成连续反馈返回给客户端
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "已完成 %.2f", feedback->progress);
            rate.sleep();
        }
        if(rclcpp::ok())
        {
            result->sum = sum;
            // 生成最终响应结果
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "已完成 100%%");
            RCLCPP_INFO(this->get_logger(), "最终结果：%d", result->sum);
        }

    }

/*AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;*/
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        // 为了避免阻塞执行器，这里需要快速返回，所以启动一个新的线程处理耗时的主逻辑实现
        std::thread{std::bind(&ProgressActionServer::execute, this, _1), goal_handle}.detach();
        
    }

private:
    rclcpp_action::Server<Progress>::SharedPtr server_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProgressActionServer>());
    rclcpp::shutdown();
    return 0;
}