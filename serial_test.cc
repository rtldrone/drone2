#include "vesc_driver/vesc_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

struct TestNode : public rclcpp::Node {
    TestNode() : Node("test_node", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor executor;
    auto test_node = std::make_shared<TestNode>();
    //auto vesc_node = std::make_shared<vesc_driver::VescDriver>(rclcpp::NodeOptions().use_intra_process_comms(true));
    //executor.add_node(test_node);

    executor.spin();
    //rclcpp::spin(std::make_shared<vesc_driver::VescDriver>(options));
    rclcpp::shutdown();
    return 0;
}