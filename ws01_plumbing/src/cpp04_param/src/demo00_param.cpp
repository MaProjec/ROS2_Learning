#include "rclcpp/rclcpp.hpp"

class Myparam : public rclcpp::Node
{
public:
  Myparam():Node("my_param_node_cpp")
  {
    RCLCPP_INFO(this->get_logger(), "参数API使用");
    // 参数对象创建
    rclcpp::Parameter p1("car_name", "tiger");
    rclcpp::Parameter p2("car_speed", 100);
    rclcpp::Parameter p3("car_weight", 1000.0);
    // 参数对象解析
    //解析值
    RCLCPP_INFO(this->get_logger(), "car_name: %s", p1.as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "car_speed: %ld", p2.as_int());
    RCLCPP_INFO(this->get_logger(), "car_weight: %.2f", p3.as_double());

    //获取参数键
    RCLCPP_INFO(this->get_logger(), "name: %s", p1.get_name().c_str());
    //获取参数值类型
    RCLCPP_INFO(this->get_logger(), "type: %s", p2.get_type_name().c_str());
    //参数值转换为字符串
    RCLCPP_INFO(this->get_logger(), "value2string: %s", p3.value_to_string().c_str());
  }
};

int main(int argc, char *argv[])
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = std::make_shared<Myparam>();

  // 运行节点
  rclcpp::spin(node);

  // 关闭
  rclcpp::shutdown();

  return 0;
}