//
// Created by cameron on 7/13/22.
//

#include <rclcpp/rclcpp.hpp>
#include "hardware_nodes/neo_3_stacklight_node.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions stacklight_options;
  stacklight_options.use_intra_process_comms(true);
  auto stacklight_node = std::make_shared<Neo3StacklightNode>("stacklight", stacklight_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(stacklight_node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}