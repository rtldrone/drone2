#include <rclcpp/rclcpp.hpp>
#include "hardware_nodes/maxbotix_ultrasonic_node.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions ultra_options;

  ultra_options.parameter_overrides().emplace_back("port", "/dev/tty.usbserial-AB6VF09X");

  auto ultra_node = std::make_shared<MaxbotixUltrasonicNode>("stacklight", ultra_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ultra_node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}