#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

class VescAdapter : public rclcpp::Node {
 public:
  explicit VescAdapter(const std::string& ns,
                       const rclcpp::NodeOptions& options);

 private:
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped> state_sub;
  rclcpp::Publisher<std_msgs::msg::Float64> speed_pub;


};
