#include "vesc_driver/vesc_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<vesc_driver::VescDriver>(options));
    rclcpp::shutdown();
    return 0;
}