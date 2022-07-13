#include "neo_3_stacklight_node.h"

#include <gpiod.h>

Neo3StacklightNode::Neo3StacklightNode(const std::string& ns, const rclcpp::NodeOptions& options)
    : Node("stacklight_node", ns, options) {
  // Initialize GPIO parameters
  auto green_chip_name = declare_parameter("green_gpio_chip_name", "gpiochip2");
  auto green_pin_number = declare_parameter("green_gpio_pin_number", 66 - 64);

  auto red_chip_name = declare_parameter("red_gpio_chip_name", "gpiochip2");
  auto red_pin_number = declare_parameter("red_gpio_pin_number", 79 - 64);

  auto blue_chip_name = declare_parameter("blue_gpio_chip_name", "gpiochip2");
  auto blue_pin_number = declare_parameter("blue_gpio_pin_number", 81 - 64);

  auto orange_chip_name = declare_parameter("orange_gpio_chip_name", "gpiochip2");
  auto orange_pin_number = declare_parameter("orange_gpio_pin_number", 82 - 64);

  // Try to open GPIO chips
  green_chip = gpiod_chip_open_by_name(green_chip_name.c_str());
  red_chip = gpiod_chip_open_by_name(red_chip_name.c_str());
  blue_chip = gpiod_chip_open_by_name(blue_chip_name.c_str());
  orange_chip = gpiod_chip_open_by_name(orange_chip_name.c_str());

  if (green_chip == nullptr || red_chip == nullptr || blue_chip == nullptr || orange_chip == nullptr) {
    RCLCPP_FATAL(get_logger(),
                 "Error opening GPIO chips!  Green open: %d  Red open: %d  Blue open: %d  Orange open: %d",
                 green_chip != nullptr, red_chip != nullptr, blue_chip != nullptr, orange_chip != nullptr);
    throw std::runtime_error("Error opening one or more GPIO chips");
  }

  // Try to open GPIO lines
  green_line = gpiod_chip_get_line(green_chip, green_pin_number);
  red_line = gpiod_chip_get_line(red_chip, red_pin_number);
  blue_line = gpiod_chip_get_line(blue_chip, blue_pin_number);
  orange_line = gpiod_chip_get_line(orange_chip, orange_pin_number);

  if (green_line == nullptr || red_line == nullptr || blue_line == nullptr || orange_line == nullptr) {
    RCLCPP_FATAL(get_logger(),
                 "Error opening GPIO lines!  Green open: %d  Red open: %d  Blue open: %d  Orange open: %d",
                 green_line != nullptr, red_line != nullptr, blue_line != nullptr, orange_line != nullptr);
    throw std::runtime_error("Error opening one or more GPIO lines");
  }

  // Reserve GPIO lines as output
  gpiod_line_request_output(green_line, get_effective_namespace().c_str(), false);
  gpiod_line_request_output(red_line, get_effective_namespace().c_str(), false);
  gpiod_line_request_output(blue_line, get_effective_namespace().c_str(), false);
  gpiod_line_request_output(orange_line, get_effective_namespace().c_str(), false);

  subscription = this->create_subscription<stacklight_state_msg::msg::StacklightState>(
      "state", 10, [this](stacklight_state_msg::msg::StacklightState::UniquePtr msg) { on_msg(*msg); });
}

Neo3StacklightNode::~Neo3StacklightNode() noexcept {
  if (green_line) gpiod_line_release(green_line);
  if (red_line) gpiod_line_release(red_line);
  if (blue_line) gpiod_line_release(blue_line);
  if (orange_line) gpiod_line_release(orange_line);

  if (green_chip) gpiod_chip_close(green_chip);
  if (red_chip) gpiod_chip_close(red_chip);
  if (blue_chip) gpiod_chip_close(blue_chip);
  if (orange_chip) gpiod_chip_close(orange_chip);
}

void Neo3StacklightNode::on_msg(const stacklight_state_msg::msg::StacklightState& msg) {
  // Update GPIO states from state
  RCLCPP_DEBUG(get_logger(), "Received state message");
  gpiod_line_set_value(green_line, msg.green);
  gpiod_line_set_value(red_line, msg.red);
  gpiod_line_set_value(blue_line, msg.blue);
  gpiod_line_set_value(orange_line, msg.orange);
}
