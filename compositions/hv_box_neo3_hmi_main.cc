#include <rclcpp/rclcpp.hpp>
#include "hmi/hmi_node.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions hmi_options;
  
  auto hmi_node = std::make_shared<HmiNode>("hmi", hmi_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(hmi_node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}