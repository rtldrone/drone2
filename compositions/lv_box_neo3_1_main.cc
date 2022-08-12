//
// Created by cameron on 7/13/22.
//

#include <rclcpp/rclcpp.hpp>

#include "hardware_nodes/maxbotix_ultrasonic_node.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions front_ultra_options;
  front_ultra_options.parameter_overrides().emplace_back("port", "/dev/ultra_front");

  auto front_ultra_node = std::make_shared<MaxbotixUltrasonicNode>("front_rangefinder", front_ultra_options);

  rclcpp::NodeOptions back_ultra_options;
  back_ultra_options.parameter_overrides().emplace_back("port", "/dev/ultra_back");

  auto back_ultra_node = std::make_shared<MaxbotixUltrasonicNode>("back_rangefinder", back_ultra_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(front_ultra_node);
  executor.add_node(back_ultra_node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}