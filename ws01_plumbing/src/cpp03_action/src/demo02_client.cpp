#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionClient : public rclcpp::Node
{
public:
    ProgressActionClient():Node("progress_action_client_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "action client constructed");

        client_ = rclcpp_action::create_client<Progress>(this,"get_sum");
    }

    //发送请求
    void send_goal(int num)
    {
        if(!client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "服务器未启动");
            return;
        }
        //发送具体请求
        auto goal = Progress::Goal();
        goal.num = num;
        auto options = rclcpp_action::Client<Progress>::SendGoalOptions();
        options.goal_response_callback = std::bind(
            &ProgressActionClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(
            &ProgressActionClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(
            &ProgressActionClient::result_callback, this, _1);
        auto future = client_->async_send_goal(goal,options);

    }
private:
    //using GoalHandle = ClientGoalHandle<ActionT>;
    //using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "目标请求被服务端拒绝");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标处理中");
        }
    }
    /*using Feedback = typename ActionT::Feedback;
    using FeedbackCallback =
    std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;*/
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle,const std::shared_ptr<const Progress::Feedback> feedback)
    {
        (void) goal_handle;
        int progress = (int) (feedback->progress * 100);
        RCLCPP_INFO(this->get_logger(), "收到反馈：%d%%", progress);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result)
    {
        if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "结果：%d", result.result->sum);
        }
        else if(result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "任务被中止");
        }
        else if(result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_ERROR(this->get_logger(), "任务被取消");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "未知异常");
        }
    }
    rclcpp_action::Client<Progress>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    if(argc != 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "请提供一个整型数据");
        return 1;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProgressActionClient>();
    node->send_goal(atoi(argv[1]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}