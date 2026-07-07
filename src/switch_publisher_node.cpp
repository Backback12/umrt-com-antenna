#include "umrt_com_antenna/switch_publisher_node.hpp"

namespace umrt_com_antenna
{

SwitchPublisherNode::SwitchPublisherNode()
: Node("switch_publisher_node"),
  chip_(GPIO_CHIP_NAME)
{
  sw1_line_ = chip_.get_line(SW1_LINE_OFFSET);
  sw2_line_ = chip_.get_line(SW2_LINE_OFFSET);

  gpiod::line_request request;
  request.consumer = "switch_publisher_node";
  request.request_type = gpiod::line_request::EVENT_BOTH_EDGES;
  // Requires libgpiod >= 1.5 / kernel >= 5.5. If unsupported on your image,
  // drop this flag and use an external pull-up resistor to 3V3 instead.
  request.flags = gpiod::line_request::FLAG_BIAS_PULL_UP;

  sw1_line_.request(request);
  sw2_line_.request(request);

  sw1_pub_ = this->create_publisher<std_msgs::msg::Bool>("switches/sw1", 10);
  sw2_pub_ = this->create_publisher<std_msgs::msg::Bool>("switches/sw2", 10);

  watch_thread_ = std::thread(&SwitchPublisherNode::watchLoop, this);

  RCLCPP_INFO(this->get_logger(), "Watching SW1 (GPIO%u) and SW2 (GPIO%u)",
    SW1_LINE_OFFSET, SW2_LINE_OFFSET);
}

SwitchPublisherNode::~SwitchPublisherNode()
{
  running_ = false;
  if (watch_thread_.joinable()) {
    watch_thread_.join();
  }
}

void SwitchPublisherNode::watchLoop()
{
//   gpiod::line_bulk lines;
//   lines.add(sw1_line_);
//   lines.add(sw2_line_);
  gpiod::line_bulk lines({sw1_line_, sw2_line_});

  while (running_ && rclcpp::ok()) {
    // event_wait blocks with a timeout so we can periodically check the running_ flag
    gpiod::line_bulk active_lines = lines.event_wait(std::chrono::milliseconds(500));
    if (!active_lines) {
      continue;  // timed out, no events
    }

    for (auto & line : active_lines) {
      gpiod::line_event event = line.event_read();
      // Switch is wired to GND: falling edge == closed/pressed, rising edge == released
      bool triggered = (event.event_type == gpiod::line_event::FALLING_EDGE);

      std_msgs::msg::Bool msg;
      msg.data = triggered;

      if (line.offset() == SW1_LINE_OFFSET) {
        sw1_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "SW1 %s", triggered ? "closed (GND)" : "released");
      } else if (line.offset() == SW2_LINE_OFFSET) {
        sw2_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "SW2 %s", triggered ? "closed (GND)" : "released");
      }
    }
  }
}

}  // namespace umrt_com_antenna