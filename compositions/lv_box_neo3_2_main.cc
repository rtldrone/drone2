//
// Created by cameron on 7/13/22.
//

#include <rclcpp/rclcpp.hpp>

#include "hardware_nodes/neo_3_stacklight_node.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions stacklight_options;
  auto stacklight_node = std::make_shared<Neo3StacklightNode>("stacklight", stacklight_options);


  rclcpp::spin(stacklight_node);
  rclcpp::shutdown();

  return 0;
}