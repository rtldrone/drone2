#include "hmi_node.h"

using json = nlohmann::json;

HmiNode::HmiNode(const std::string& ns, const rclcpp::NodeOptions& options) : rclcpp::Node("hmi", ns, options) {
  input_subscription = create_subscription<hmi_msgs::msg::HmiInputState>(
      "input", 10, [this](hmi_msgs::msg::HmiInputState::UniquePtr msg) { on_input(*msg); });
  output_publisher = create_publisher<hmi_msgs::msg::HmiOutputState>("output", 10);

  stop_client = create_client<std_srvs::srv::Trigger>("stop");
  speed_up_client = create_client<std_srvs::srv::Trigger>("speed_up");
  speed_down_client = create_client<std_srvs::srv::Trigger>("speed_down");

  // Start the websocket server
  server.init_asio();
  server.set_access_channels(websocketpp::log::alevel::none);  // Disable access logs
  server.set_reuse_addr(true);  // This lets us reuse the socket if the process restarts fast
  server.set_validate_handler([this](websocketpp::connection_hdl hdl) { return on_validate(hdl); });
  server.set_fail_handler([](websocketpp::connection_hdl hdl) {});
  server.set_close_handler([](websocketpp::connection_hdl hdl) {});
  server.set_message_handler(
      [this](websocketpp::connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr ptr) {
        on_message(hdl, ptr);
      });

  server.listen(8082);
  server.start_accept();

  websocket_thread = std::thread([this]() { server.run(); });
}

// Called whenever we get a ROS message to update the HMI inputs.
// This converts the message to JSON and sends it to the tablet.
void HmiNode::on_input(const hmi_msgs::msg::HmiInputState& msg) {
  std::lock_guard<std::mutex> guard(connection_mtx);

  json data = {{"diagnostics", msg.diagnostics},
               {"speed_setpoint", msg.speed_setpoint},
               {"current_speed", msg.current_speed},
               {"voltage", msg.voltage},
               {"motor_current_draw", msg.motor_current_draw},
               {"distance", msg.distance},
               {"battery_state", msg.battery_state},
               {"motor_current_state", msg.motor_current_state},
               {"distance_state", msg.distance_state}};

  if (current_connection.has_value()) {
    websocketpp::lib::error_code ec;
    server.send(current_connection.value(), data.dump(), websocketpp::frame::opcode::text, ec);
  }
}

// Called when a new websocket connection happens.  This can be
// when the tablet first starts or reconnects.
// We just close the previous connection and replace it with the new one.
bool HmiNode::on_validate(websocketpp::connection_hdl hdl) {
  std::lock_guard<std::mutex> guard(connection_mtx);
  if (current_connection.has_value()) {
    websocketpp::lib::error_code ec;
    server.close(current_connection.value(), websocketpp::close::status::normal, "New connection established", ec);
    RCLCPP_INFO(get_logger(), "Disconnected old client\n");
  }
  current_connection = hdl;
  RCLCPP_INFO(get_logger(), "New client connected\n");
  return true;
}

// For a periodic message, we just unpack the json and publish a ROS message.
void HmiNode::handle_periodic(json& data) {
  try {
    auto msg = std::make_unique<hmi_msgs::msg::HmiOutputState>();
    msg->forward = data["forward"];
    msg->disable_sensors = data["disable_sensors"];

    // Publish the message
    output_publisher->publish(std::move(msg));
  } catch (json::type_error &e) {
    RCLCPP_ERROR(get_logger(), "Bad json data for periodic: %s\n", data.dump().c_str());
  }
}

// For a command message, we have to figure out the command and call a service based on that
void HmiNode::handle_command(json& data) {
  try {
    if (data["command"] == "stop") {
      // Call the stop service
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      stop_client->async_send_request(req);
    } else if (data["command"] == "speed_up") {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      speed_up_client->async_send_request(req);
    } else if (data["command"] == "speed_down") {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      speed_down_client->async_send_request(req);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown command type: %s\n", data["command"].get<std::string>().c_str());
    }
  } catch (json::type_error &e) {
    RCLCPP_ERROR(get_logger(), "Bad json data for command: %s\n", data.dump().c_str());
  }
}

// Called whenever a websocket message arrives from the tablet.  We determine
// if it is periodic or a command, and then fan out control to the appropriate location.
void HmiNode::on_message(websocketpp::connection_hdl hdl,
                         websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
  try {
    json data = json::parse(msg->get_payload());
    if (data["type"] == "periodic") {
      handle_periodic(data);
    } else if (data["type"] == "command") {
      handle_command(data);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown message type: %s\n", data["type"].get<std::string>().c_str());
    }
  } catch (json::parse_error& e) {
    RCLCPP_ERROR(get_logger(), "Error parsing WS data from client: %s\n", msg->get_payload().c_str());
  } catch (json::type_error& e) {
    RCLCPP_ERROR(get_logger(), "Bad data from client: %s\n", msg->get_payload().c_str());
  }
}