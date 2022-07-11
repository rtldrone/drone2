#pragma once

#include <gpiod.h>

#include "rclcpp/rclcpp.hpp"
#include "stacklight_state_msg/msg/stacklight_state.hpp"

class Neo3StacklightNode : public rclcpp::Node {
 public:
  Neo3StacklightNode(const std::string& ns, const rclcpp::NodeOptions& options);

 private:
  void on_msg(const stacklight_state_msg::msg::StacklightState& msg);
  rclcpp::Subscription<stacklight_state_msg::msg::StacklightState>::SharedPtr
      subscription;
};
