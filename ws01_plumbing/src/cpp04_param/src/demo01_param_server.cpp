#include "rclcpp/rclcpp.hpp"

class ParamServer: public rclcpp::Node {
public:
    ParamServer(): Node("param_server_node_cpp",
                            rclcpp::NodeOptions().allow_undeclared_parameters(true)) {
        RCLCPP_INFO(this->get_logger(), "参数服务端创建成功");
    }
    void declare_param(){
        RCLCPP_INFO(this->get_logger(), "开始声明参数");
        //声明参数
        this->declare_parameter("car_name", "tiger");
        this->declare_parameter("car_speed", 100);
        this->declare_parameter("car_weight", 1000.0);

        //也可以用set,但必须保证rclcpp::NodeOptions().allow_undeclared_parameters(true)
        this->set_parameter(rclcpp::Parameter("car_height", 2.0));
    }
    void get_param(){
        RCLCPP_INFO(this->get_logger(), "开始获取参数");
        //获取指定参数
        auto car = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(), "key: %s,value:%s", car.get_name().c_str(),car.as_string().c_str());
        //获取一些参数
        auto params = this->get_parameters({"car_name","car_speed", "car_weight"});
        for(auto &&param : params){
            RCLCPP_INFO(this->get_logger(), "key: %s,value:%s", param.get_name().c_str(),param.value_to_string().c_str());
        }
        //判断是否包含
        RCLCPP_INFO(this->get_logger(), "是否包含car_name: %d", this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "是否包含car_color: %d", this->has_parameter("car_color"));

        
    }
    void update_param(){
        RCLCPP_INFO(this->get_logger(), "开始更新参数");
        //更新参数
        this->set_parameter(rclcpp::Parameter("car_name", "lion"));
        this->set_parameter(rclcpp::Parameter("car_speed", 200));
        this->set_parameter(rclcpp::Parameter("car_weight", 2000.0));
        //打印更新后的参数
        auto params = this->get_parameters({"car_name","car_speed", "car_weight","car_height"});
        for(auto &&param : params){
            RCLCPP_INFO(this->get_logger(), "key: %s,value:%s", param.get_name().c_str(),param.value_to_string().c_str());
        }
    }
    void del_param(){
        RCLCPP_INFO(this->get_logger(), "开始删除参数");
        //删除参数
        //this->undeclare_parameter("car_name");不能删除声明参数
        RCLCPP_INFO(this->get_logger(), "删除前是否包含car_name: %d", this->has_parameter("car_height"));
        this->undeclare_parameter("car_height");//可以删除未声明然后设置的参数
        RCLCPP_INFO(this->get_logger(), "删除后是否包含car_name: %d", this->has_parameter("car_height"));
    }
private:

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParamServer>();
    node->declare_param();
    node->get_param();
    node->update_param();
    node->del_param();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}