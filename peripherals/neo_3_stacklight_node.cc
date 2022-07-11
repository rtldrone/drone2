#include "neo_3_stacklight_node.h"

Neo3StacklightNode::Neo3StacklightNode(const std::string& ns,
                                       const rclcpp::NodeOptions& options)
    : Node(ns, options) {
  subscription =
      this->create_subscription<stacklight_state_msg::msg::StacklightState>(
          "red", 10,
          std::bind(&Neo3StacklightNode::on_msg, this, std::placeholders::_1));


}

void Neo3StacklightNode::on_msg(const stacklight_state_msg::msg::StacklightState& msg) {
  gpiod_line_get()
}
