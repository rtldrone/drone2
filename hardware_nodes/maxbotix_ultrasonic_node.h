#pragma once

#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <thread>
#include <functional>

#include "sensor_msgs/msg/range.hpp"

class MaxbotixUltrasonicNode : public rclcpp::Node {
 public:
  MaxbotixUltrasonicNode(const std::string& ns, const rclcpp::NodeOptions& options);
  //~MaxbotixUltrasonicNode() noexcept override;

 private:
  asio::io_context io;
  asio::serial_port port;
  asio::streambuf buffer;
  std::thread serial_thread;
  int error_count = 0;

  const std::function<void(const asio::error_code&, std::size_t)> read_callback;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> range_publisher;

  void on_read(const asio::error_code& error, std::size_t bytes_received);
};
