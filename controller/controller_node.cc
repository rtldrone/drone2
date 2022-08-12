#include "controller_node.h"

using namespace std::chrono_literals;

ControllerNode::ControllerNode(const std::string& ns, const rclcpp::NodeOptions& options)
    : rclcpp::Node("controller", ns, options),
      profiler(VEHICLE_ACCEL_M_PER_S2),
      red_blinker(10),
      green_blinker(10),
      orange_blinker(10),
      blue_blinker(10) {
  using hmi_msgs::msg::HmiInputState;
  using hmi_msgs::msg::HmiOutputState;
  using sensor_msgs::msg::Range;
  using stacklight_state_msg::msg::StacklightState;
  using std_msgs::msg::Float64;
  using std_srvs::srv::Trigger;
  using vesc_msgs::msg::VescState;
  using vesc_msgs::msg::VescStateStamped;
  front_range_subscription = create_subscription<Range>(
      "/front_rangefinder/range", 10, [this](std::unique_ptr<Range> msg) { front_range_m = msg->range; });
  back_range_subscription = create_subscription<Range>(
      "/back_rangefinder/range", 10, [this](std::unique_ptr<Range> msg) { back_range_m = msg->range; });
  vesc_state_subscription =
      create_subscription<VescStateStamped>("/motor/sensors/core", 10, [this](std::unique_ptr<VescStateStamped> msg) {
        vesc_fault = msg->state.fault_code != VescState::FAULT_CODE_NONE;
        speed_m_per_s = erpm_to_m_per_s(msg->state.speed);
        current_draw_a = msg->state.current_motor;
        battery_voltage = msg->state.voltage_input;
      });
  hmi_output_subscription =
      create_subscription<HmiOutputState>("/hmi/output", 10, [this](std::unique_ptr<HmiOutputState> msg) {
        hmi_forward = msg->forward;
        hmi_sensor_override = msg->disable_sensors;
        last_hmi_update = std::chrono::steady_clock::now();
      });
  hmi_stop_service = create_service<Trigger>(
      "/hmi/stop", [this](std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response) {
        hmi_goal_mph = 0;
        response->success = true;
      });
  hmi_speed_up_service = create_service<Trigger>(
      "/hmi/speed_up", [this](std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response) {
        if (hmi_goal_mph == 0)  // Prevent going to 1 mph
          hmi_goal_mph = 2;
        else
          hmi_goal_mph++;
        if (hmi_goal_mph > MAX_SPEED_MPH) hmi_goal_mph = MAX_SPEED_MPH;
        clear_stop_conditions();
        response->success = true;
      });
  hmi_speed_down_service = create_service<Trigger>(
      "/hmi/speed_down",
      [this](std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response) {
        if (hmi_goal_mph == 2)  // Prevent going to 1 mph
          hmi_goal_mph = 0;
        else
          hmi_goal_mph--;
        if (hmi_goal_mph < 0) hmi_goal_mph = 0;
        response->success = true;
      });

  vesc_speed_publisher = create_publisher<Float64>("/motor/commands/motor/speed", 10);
  stacklight_state_publisher = create_publisher<StacklightState>("/stacklight/state", 10);
  hmi_input_publisher = create_publisher<HmiInputState>("/hmi/input", 10);

  timer = create_wall_timer(UPDATE_RATE, [this]() { on_timer(); });
}

void ControllerNode::on_timer() {
  // Update estop
  if (is_estopped) {
    stopped_due_to_estop = true;
  }

  // Decide which distance sensor to read from the callback
  double range_m = hmi_forward ? front_range_m : back_range_m;
  range_m -= DISTANCE_OFFSET_M;
  if (range_m < 0) range_m = 0;
  double direction_sign = hmi_forward ? 1.0 : -1.0;

  // Update the velocity profiler
  profiler.set_goal(direction_sign * mph_to_m_per_s(hmi_goal_mph));

  // Check stop conditions
  double stopping_distance = profiler.get_stopping_distance();
  if (!hmi_sensor_override && range_m <= stopping_distance) {
    stopped_due_to_distance = true;
    profiler.set_goal(0);
    hmi_goal_mph = 0;
  }

  auto now = std::chrono::steady_clock::now();
  if (now - last_hmi_update > HMI_TIMEOUT) {
    stopped_due_to_loss_of_hmi = true;
    profiler.set_goal(0);
    hmi_goal_mph = 0;
  }

  profiler.update(UPDATE_RATE.count() / 1000.0);

  // Send command to motor
  double motor_erpm_target = m_per_s_to_erpm(profiler.get_setpoint());
  auto motor_message = std::make_unique<std_msgs::msg::Float64>();
  motor_message->data = motor_erpm_target;
  vesc_speed_publisher->publish(std::move(motor_message));

  // Send data to HMI
  auto hmi_message = std::make_unique<hmi_msgs::msg::HmiInputState>();
  if (hmi_sensor_override) {
    hmi_message->diagnostics.push_back("WARNING: Distance sensors are DISABLED");
  }
  if (stopped_due_to_distance) {
    hmi_message->diagnostics.push_back("The vehicle was stopped due to an object detected within an unsafe distance");
  }
  if (stopped_due_to_loss_of_hmi) {
    hmi_message->diagnostics.push_back("The vehicle was stopped due to a loss of HMI connection");
  }
  if (stopped_due_to_estop) {
    hmi_message->diagnostics.push_back("The vehicle was stopped due to an E-Stop activation");
  }

  hmi_message->speed_setpoint = hmi_goal_mph;
  hmi_message->current_speed = std::abs(m_per_s_to_mph(speed_m_per_s));
  hmi_message->voltage = battery_voltage;
  hmi_message->motor_current_draw = current_draw_a;
  hmi_message->distance = m_to_ft(range_m);
  hmi_message->distance_state = get_distance_state(profiler.get_stopping_distance(), range_m);
  hmi_message->motor_current_state = get_current_draw_state();
  hmi_message->battery_state = get_battery_state();
  hmi_input_publisher->publish(std::move(hmi_message));

  // Update stacklight
  red_blinker.update();
  green_blinker.update();
  orange_blinker.update();
  blue_blinker.update();

  if (hmi_goal_mph == 0) {
    red_blinker.set_mode(Blinker::ON);
    green_blinker.set_mode(Blinker::OFF);
    blue_blinker.set_mode(Blinker::OFF);
  } else if (hmi_sensor_override) {
    red_blinker.set_mode(Blinker::OFF);
    green_blinker.set_mode(Blinker::ON);
    blue_blinker.set_mode(Blinker::OFF);
  } else {
    red_blinker.set_mode(Blinker::OFF);
    green_blinker.set_mode(Blinker::ON);
    blue_blinker.set_mode(Blinker::BLINK);
  }

  if (hmi_sensor_override) {
    orange_blinker.set_mode(Blinker::BLINK);
  } else {
    orange_blinker.set_mode(Blinker::OFF);
  }

  if (is_estopped) {
    red_blinker.set_mode(Blinker::BLINK);
    green_blinker.set_mode(Blinker::OFF);
    orange_blinker.set_mode(Blinker::OFF);
    blue_blinker.set_mode(Blinker::OFF);
  }

  auto light_msg = std::make_unique<stacklight_state_msg::msg::StacklightState>();
  light_msg->red = red_blinker.get_status();
  light_msg->green = green_blinker.get_status();
  light_msg->blue = blue_blinker.get_status();
  light_msg->orange = orange_blinker.get_status();
  stacklight_state_publisher->publish(std::move(light_msg));
}