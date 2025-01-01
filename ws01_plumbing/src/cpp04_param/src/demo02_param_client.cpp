#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
class ParamClient : public rclcpp::Node
{
public:
    ParamClient() : Node("param_client_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "param_client created successfully.");
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "param_server_node_cpp");
    }
    bool connect_sever()
    {
        while(!param_client_->wait_for_service(1s)){
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        return true;
    }
    void get_param()
    {
        RCLCPP_INFO(this->get_logger(), "start getting parameters");
        //获取某个参数
        std::string car_name = param_client_->get_parameter<std::string>("car_name");
        double car_weight = param_client_->get_parameter<double>("car_weight");
        RCLCPP_INFO(this->get_logger(), "car_name: %s, car_weight: %f", car_name.c_str(), car_weight);
        //获取多个参数
        auto params = param_client_->get_parameters({"car_name", "car_speed", "car_weight"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "key: %s, value: %s", param.get_name().c_str(), param.value_to_string().c_str());
        }

        //判断是否包含
        RCLCPP_INFO(this->get_logger(), "has car_name? %d", param_client_->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "has car_color? %d", param_client_->has_parameter("car_color"));
        
    }
    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "start updating parameters");
        //更新参数
        param_client_->set_parameters({rclcpp::Parameter("car_name", "turtle"),
                                       rclcpp::Parameter("car_speed", 300),
                                       rclcpp::Parameter("car_weight", 3000.0)});
        param_client_->set_parameters({rclcpp::Parameter("car_color", "red")});
        //打印更新后的参数
        auto params = param_client_->get_parameters({"car_name", "car_speed", "car_weight", "red"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "key: %s, value: %s", param.get_name().c_str(), param.value_to_string().c_str());
        }

    }

private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ParamClient>();
    if(!client->connect_sever()){
        return 0;
    }
    client->get_param();
    client->update_param();
    client->get_param();
    rclcpp::shutdown();
    return 0;
}