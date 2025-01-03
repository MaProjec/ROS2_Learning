#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node","t1_ns") {
    RCLCPP_INFO(this->get_logger(), "Node created successfully!");
  }

private:
  // Add private members here
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}