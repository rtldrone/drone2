#include <rclcpp/rclcpp.hpp>

#include "controller/controller_node.h"
#include "hardware_nodes/neo_3_safety_node.h"
#include "hmi/hmi_node.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions controller_options;

  auto controller_node = std::make_shared<ControllerNode>("controller", controller_options);

  rclcpp::NodeOptions hmi_options;

  auto hmi_node = std::make_shared<HmiNode>("hmi", hmi_options);

  rclcpp::NodeOptions safety_options;

  auto safety_node = std::make_shared<Neo3SafetyNode>("safety", safety_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(safety_node);
  executor.add_node(controller_node);
  executor.add_node(hmi_node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}