#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using std::placeholders::_1;
using std::placeholders::_2;

class AddIntsServer : public rclcpp::Node
{
public:
  AddIntsServer () : Node("add_ints_server_node_cpp")
  {
    RCLCPP_INFO(this->get_logger(), "AddIntsServer constructed");
    server_ = this->create_service<AddInts>(
      "add_ints", std::bind(&AddIntsServer::add, this, _1, _2));
  }

private:
void add(const AddInts::Request::SharedPtr request,
         AddInts::Response::SharedPtr response)
         {
            response->sum = request->num1 + request->num2;
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", request->num1, request->num2, response->sum);
         }  
  rclcpp::Service<AddInts>::SharedPtr server_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddIntsServer >());
  rclcpp::shutdown();
  return 0;
}
