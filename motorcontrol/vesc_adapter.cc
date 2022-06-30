#include "vesc_adapter.h"

VescAdapter::VescAdapter(const std::string& ns,
                         const rclcpp::NodeOptions& options)
    : Node(ns, options) {}
