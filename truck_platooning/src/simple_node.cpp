#include "rclcpp/rclcpp.hpp"

class SimpleNode : public rclcpp::Node {
public:
  SimpleNode() : Node("simple_node") {
    RCLCPP_INFO(this->get_logger(), "Hello from C++ ROS2 node!");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNode>());
  rclcpp::shutdown();
  return 0;
}

