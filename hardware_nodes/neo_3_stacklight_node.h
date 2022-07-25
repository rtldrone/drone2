#pragma once

#include <gpiod.h>

#include "rclcpp/rclcpp.hpp"
#include "stacklight_state_msg/msg/stacklight_state.hpp"

class Neo3StacklightNode : public rclcpp::Node {
 public:
  Neo3StacklightNode(const std::string& ns, const rclcpp::NodeOptions& options);
  ~Neo3StacklightNode() noexcept override;

 private:
  gpiod_chip* green_chip;
  gpiod_chip* red_chip;
  gpiod_chip* blue_chip;
  gpiod_chip* orange_chip;

  gpiod_line* green_line;
  gpiod_line* red_line;
  gpiod_line* blue_line;
  gpiod_line* orange_line;

  [[maybe_unused]] std::shared_ptr<rclcpp::Subscription<stacklight_state_msg::msg::StacklightState>> subscription;

  void on_msg(const stacklight_state_msg::msg::StacklightState& msg);
};
