
#include <rclcpp/rclcpp.hpp>
#include <vesc_driver/vesc_driver.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions vesc_driver_options;
  vesc_driver_options.parameter_overrides().emplace_back("port", "/dev/ttyACM0");
  vesc_driver_options.parameter_overrides().emplace_back("speed_min", -100000.0);
  vesc_driver_options.parameter_overrides().emplace_back("speed_max", 100000.0);

  auto vesc_driver_node = std::make_shared<vesc_driver::VescDriver>("motor", vesc_driver_options);

  rclcpp::spin(vesc_driver_node);
  rclcpp::shutdown();

  return 0;
}