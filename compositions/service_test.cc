#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

void test(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // ...
  std::cout << "service called" << std::endl;
  response->success = true;

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("service_test");

  auto service = node->create_service<std_srvs::srv::Trigger>("/hmi/stop", &test);

  rclcpp::spin(node);
  rclcpp::shutdown();
}