#include "umrt_com_antenna/switch_publisher_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<umrt_com_antenna::SwitchPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}