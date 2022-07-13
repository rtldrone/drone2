#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "stacklight_state_msg/msg/stacklight_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
      publisher_ = create_publisher<stacklight_state_msg::msg::StacklightState>("/stacklight/state", 10);
        auto timer_callback = [this]() -> void {
            RCLCPP_INFO(get_logger(), "Starting publish");
            auto message = std::make_unique<stacklight_state_msg::msg::StacklightState>();
            message->orange = state;
            state = !state;
            RCLCPP_INFO(get_logger(), "Publishing");
            publisher_->publish(std::move(message));
            RCLCPP_INFO(get_logger(), "Publisih done");
        };
        timer_ = create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<stacklight_state_msg::msg::StacklightState>::SharedPtr publisher_;
    size_t count_;
    bool state = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}