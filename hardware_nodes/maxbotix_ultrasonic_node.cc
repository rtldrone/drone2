#include "maxbotix_ultrasonic_node.h"

MaxbotixUltrasonicNode::MaxbotixUltrasonicNode(const std::string& ns, const rclcpp::NodeOptions& options)
    : rclcpp::Node("maxbotix_ultrasonic", ns, options),
      port(io),
      buffer(16),
      read_callback([this](auto& error_code, auto bytes_received) { on_read(error_code, bytes_received); }) {
  auto port_name = declare_parameter<std::string>("port");

  try {
    port.open(port_name);
    port.set_option(asio::serial_port::baud_rate(9600));
  } catch (asio::system_error& e) {
    RCLCPP_FATAL(get_logger(), "Error opening serial port '%s'", port_name.c_str());
    throw std::runtime_error("Error opening serial port");
  }

  // Create the publisher
  range_publisher = create_publisher<sensor_msgs::msg::Range>("range", 10);

  // Initiate the first read
  // This needs to happen before we start the thread, otherwise the queue will
  // immediately run out of tasks
  asio::async_read_until(port, buffer, (char)13, read_callback);

  // Start the background thread
  serial_thread = std::thread([this]() { io.run(); });
}

void MaxbotixUltrasonicNode::on_read(const asio::error_code& error, std::size_t bytes_received) {
  auto capture_time = now();

  bool published = false;

  if (error) {
    RCLCPP_WARN(get_logger(), "Error reading from sensor: %s\n", error.message().c_str());
  } else {
    std::string data(asio::buffers_begin(buffer.data()),
                     asio::buffers_begin(buffer.data()) + (std::ptrdiff_t)bytes_received);
    // Check that the string starts with 'R'
    if (data.find_first_not_of('R') != 1) {
      RCLCPP_WARN(get_logger(), "Got malformed string from sensor: %s\n", data.c_str());
    } else {
      // Extract number from string and publish a message
      try {
        int cm = std::stoi(data.substr(1));

        auto message = std::make_unique<sensor_msgs::msg::Range>();
        message->header.stamp = capture_time;
        message->radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        message->field_of_view = M_PI / 6.0;
        message->min_range = .2;
        message->max_range = 7.65;
        message->range = (float)cm / 100.0f;
        if (message->range > message->max_range) message->range = std::numeric_limits<float>::infinity();
        if (message->range < message->min_range) message->range = -std::numeric_limits<float>::infinity();
        RCLCPP_DEBUG(get_logger(), "Publishing distance: %f m\n", message->range);
        range_publisher->publish(std::move(message));
        published = true;
      } catch (std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to read number from sensor data: %s\n", data.substr(1).c_str());
      }
    }
  }

  buffer.consume(bytes_received);
  if (!published) {
    error_count++;
    if (error_count > 20) {
      // Crash the process as we've had a large number of errors in a row.
      RCLCPP_FATAL(get_logger(), "Too many errors, exiting!\n");
      throw std::runtime_error("Too many errors reading ultrasonic");
    }
    // Something didn't work right, add a 100 ms delay before trying again to keep us
    // from slamming the thread at 100% if things keep not working.
    io.dispatch([]() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); });
  } else {
    error_count = 0;
  }
  asio::async_read_until(port, buffer, (char)13, read_callback);
}