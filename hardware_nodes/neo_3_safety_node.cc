#include "neo_3_safety_node.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

Neo3SafetyNode::Neo3SafetyNode(const std::string& ns, const rclcpp::NodeOptions& options) : rclcpp::Node(ns, options) {
  i2c_fd = open("/dev/i2c-0", O_RDWR);

  if (i2c_fd < 0) {
    RCLCPP_FATAL(get_logger(), "Error opening i2c!\n");
    throw std::runtime_error("error opening i2c");
  }

  if (ioctl(i2c_fd, I2C_SLAVE, 0x42) < 0) {
    RCLCPP_FATAL(get_logger(), "Error setting i2c slave!\n");
    throw std::runtime_error("ioctl error");
  }

  using std_srvs::srv::Trigger;

  safety_update_service = create_service<Trigger>(
      "update", [this](std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response) {
        char buf[1];
        if (read(i2c_fd, buf, 1) != 1) {
          RCLCPP_WARN(get_logger(), "Could not read safety state\n");
          response->success = false;
        }
        response->success = buf[0] == 0;
      });
}

Neo3SafetyNode::~Neo3SafetyNode() {
  if (i2c_fd) close(i2c_fd);
}