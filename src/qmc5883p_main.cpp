#include "umrt_com_antenna/qmc5883p_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<umrt_com_antenna::Qmc5883pNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}