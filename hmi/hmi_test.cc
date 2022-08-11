#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "third_party/json/json.hpp"

#include <optional>
using json = nlohmann::json;

websocketpp::server<websocketpp::config::asio> server;

std::optional<websocketpp::connection_hdl> current_connection;

json my_json = {{"message", "test"}, {"value", 2}};

int speed = 0;

bool on_validate(websocketpp::connection_hdl hdl) {
  if (current_connection.has_value()) {
    websocketpp::lib::error_code ec;
    server.close(current_connection.value(), websocketpp::close::status::normal, "New connection established", ec);
    std::cout << "Closed old connection" << std::endl;
  }
  current_connection = hdl;
  std::cout << "New client connected" << std::endl;
  return true;
}

void on_fail(websocketpp::connection_hdl hdl) {
  std::cout << "fail" << std::endl;
}

void on_close(websocketpp::connection_hdl hdl) {
  std::cout << "closed" << std::endl;
}

void on_message(websocketpp::connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
  try {
    json data = json::parse(msg->get_payload());
    if (data["type"] == "periodic") {

    }
    if (data["type"] == "command") {
      if (data["command"] == "speed_up") {
        speed++;
      }
      if (data["command"] == "speed_down") {
        speed--;
        if (speed < 0) speed = 0;
      }
    }
  } catch (json::parse_error &e) {

  }
  websocketpp::lib::error_code ec;
  json response = {
    {"diagnostics", json::array({"Example diagnostic"})},
    {"speed_setpoint", speed},
    {"current_speed", std::rand() % 25},
    {"voltage", std::rand() % 30},
    {"motor_current_draw", std::rand() % 10},
    {"distance", std::rand() % 500},
    {"battery_state", std::rand() % 3},
    {"motor_current_state", std::rand() % 3},
    {"distance_state", std::rand() % 3}
  };
  server.send(hdl, response.dump(), websocketpp::frame::opcode::text, ec);
}

int main() {
  server.init_asio();

  server.set_access_channels(websocketpp::log::alevel::none);
  server.set_reuse_addr(true);
  server.set_validate_handler(&on_validate);
  server.set_fail_handler(&on_fail);
  server.set_close_handler(&on_close);
  server.set_message_handler(&on_message);
  server.listen(8082);
  websocketpp::lib::error_code ec;
  server.start_accept(ec);

  server.run();
}