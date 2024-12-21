#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;
class ListenerStu : public rclcpp::Node
{
public:
    ListenerStu() : Node("listenerstu_node_cpp")
    {
        subscription_ = this->create_subscription<Student>("chatter_stu", 10, std::bind(&ListenerStu::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const Student &stu)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s,%d,%.2f'", stu.name.c_str(),stu.age,stu.height);
    }
    rclcpp::Subscription<Student>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerStu>());
    rclcpp::shutdown();
    return 0;
}
