#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

class AddIntsClient : public rclcpp::Node
{
public:
    AddIntsClient() : rclcpp::Node("add_ints_client_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "AddIntsClient constructed");
        client_ = this->create_client<AddInts>("add_ints");
    }
    bool connect_server()
    {
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "client interrupted while waiting for service to appear.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务连接中...");
        }
        
        return true;
    }
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1, int num2)
    {
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
       //发送请求
        return client_->async_send_request(request);
   
    }
private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "清提交两个整型数据");
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "提交两个整型数据");
        return 1;
    }
    rclcpp::init(argc, argv);
    auto client = std::make_shared<AddIntsClient>();
    bool flag = client->connect_server();
    if(!flag)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "服务未连接成功");
        return 0;
    }
    auto response = client->send_request(atoi(argv[1]), atoi(argv[2]));
    if (rclcpp::spin_until_future_complete(client, response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(), "Sum: %d", response.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(client->get_logger(), "服务响应失败");
    }
    rclcpp::shutdown();
    return 0;
}