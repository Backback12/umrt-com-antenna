#include "umrt_com_antenna/mpu9265_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<umrt_com_antenna::Mpu9265Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}