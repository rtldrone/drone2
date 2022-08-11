#pragma once

#include <rclcpp/rclcpp.hpp>
#include <hmi_msgs/msg/hmi_input_state.hpp>
#include <hmi_msgs/msg/hmi_output_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <websocketpp/server.hpp>
#include <websocketpp/config/asio_no_tls.hpp>

#include <optional>
#include <thread>
#include "third_party/json/json.hpp"

class HmiNode : public rclcpp::Node {
  public:
  HmiNode(const std::string& ns, const rclcpp::NodeOptions& options);
  //~HmiNode() noexcept override;

  private:
  [[maybe_unused]] std::shared_ptr<rclcpp::Subscription<hmi_msgs::msg::HmiInputState>> input_subscription;
  std::shared_ptr<rclcpp::Publisher<hmi_msgs::msg::HmiOutputState>> output_publisher;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> stop_client;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> speed_up_client;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> speed_down_client;

  void on_input(const hmi_msgs::msg::HmiInputState& msg);

  bool on_validate(websocketpp::connection_hdl hdl);
  void on_message(websocketpp::connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg);

  void handle_periodic(nlohmann::json& data);
  void handle_command(nlohmann::json& data);

  websocketpp::server<websocketpp::config::asio> server;
  std::optional<websocketpp::connection_hdl> current_connection;
  [[maybe_unused]] std::thread websocket_thread;

  std::mutex connection_mtx;
};