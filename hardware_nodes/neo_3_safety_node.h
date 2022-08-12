#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>


class Neo3SafetyNode : public rclcpp::Node {
    public:
    Neo3SafetyNode(const std::string& ns, const rclcpp::NodeOptions& options);
    ~Neo3SafetyNode() noexcept override;

    private:
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> safety_update_service;

    int i2c_fd;

};