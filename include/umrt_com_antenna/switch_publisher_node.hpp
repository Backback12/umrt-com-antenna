#pragma once

#include <atomic>
#include <thread>

#include <gpiod.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace umrt_com_antenna
{

/**
 * @brief Publishes on GPIO edge events for two momentary switches wired to GND.
 *
 * SW1 -> GPIO17 (BCM), SW2 -> GPIO27 (BCM), both configured with internal pull-ups.
 * A switch closing to GND produces a falling edge; opening produces a rising edge.
 */
class SwitchPublisherNode : public rclcpp::Node
{
public:
  SwitchPublisherNode();
  ~SwitchPublisherNode() override;

private:
  void watchLoop();

  static constexpr unsigned int SW1_LINE_OFFSET = 17;
  static constexpr unsigned int SW2_LINE_OFFSET = 27;
  static constexpr const char * GPIO_CHIP_NAME = "gpiochip0";

  gpiod::chip chip_;
  gpiod::line sw1_line_;
  gpiod::line sw2_line_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sw1_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sw2_pub_;

  std::thread watch_thread_;
  std::atomic<bool> running_{true};
};

}  // namespace umrt_com_antenna