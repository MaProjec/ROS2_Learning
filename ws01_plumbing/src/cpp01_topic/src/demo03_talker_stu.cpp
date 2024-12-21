#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;
class TalkerStu : public rclcpp::Node
{
public:
    TalkerStu() : Node("talkerstu_node_cpp"),age(0)
    {
        publisher_ = this->create_publisher<Student>("chatter_stu", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TalkerStu::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto stu = Student();
        stu.name = "葫芦娃";
        stu.age = age;
        stu.height = 2.20;
        age++;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s,%d,%.2f'", stu.name.c_str(),stu.age,stu.height);
        publisher_->publish(stu);
    }
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int age = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerStu>());
    rclcpp::shutdown();
    return 0;
}