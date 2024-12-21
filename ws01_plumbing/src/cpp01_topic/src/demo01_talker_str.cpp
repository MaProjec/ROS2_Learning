
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker: public rclcpp::Node
{
public:
  Talker(): Node("talker_node_cpp"),count(0)
  {
    RCLCPP_INFO(this->get_logger(), "talker_node_cpp created");
    publisher1 = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer1 = this->create_wall_timer(1s,std::bind(&Talker::timer_callback, this));
  }

  private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world!" + std::to_string(count++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher1->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1;
  rclcpp::TimerBase::SharedPtr timer1;
  size_t count = 0;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  
  rclcpp::shutdown();
  return 0;
}
