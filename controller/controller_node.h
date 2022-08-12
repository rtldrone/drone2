#pragma once

#include <chrono>
#include <stdint.h>

#include <hmi_msgs/msg/hmi_input_state.hpp>
#include <hmi_msgs/msg/hmi_output_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <stacklight_state_msg/msg/stacklight_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "velocity_profiler.h"
#include "blinker.h"

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode(const std::string& ns, const rclcpp::NodeOptions& options);

 private:
  // Front and back rangefinders
  [[maybe_unused]] std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Range>> front_range_subscription;
  [[maybe_unused]] std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Range>> back_range_subscription;

  // VESC core state + speed publisher
  [[maybe_unused]] std::shared_ptr<rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>> vesc_state_subscription;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> vesc_speed_publisher;

  // Stacklight
  std::shared_ptr<rclcpp::Publisher<stacklight_state_msg::msg::StacklightState>> stacklight_state_publisher;

  // HMI
  [[maybe_unused]] std::shared_ptr<rclcpp::Subscription<hmi_msgs::msg::HmiOutputState>> hmi_output_subscription;
  std::shared_ptr<rclcpp::Publisher<hmi_msgs::msg::HmiInputState>> hmi_input_publisher;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> hmi_stop_service;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> hmi_speed_up_service;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> hmi_speed_down_service;

  std::shared_ptr<rclcpp::TimerBase> timer;

  void on_timer();

  Blinker red_blinker;
  Blinker green_blinker;
  Blinker orange_blinker;
  Blinker blue_blinker;

  inline double m_to_ft(double m) {
    return m * 3.28084;
  }

  inline double erpm_to_m_per_s(double erpm) {
    return erpm * 2.0 * M_PI * WHEEL_RADIUS_M / 7.0 / 60.0;
  }

  inline double m_per_s_to_erpm(double m_per_s) {
    return m_per_s / 2.0 / M_PI / WHEEL_RADIUS_M * 7.0 * 60.0;
  }

  inline double m_per_s_to_mph(double m_per_s) {
    return m_per_s * 2.23694;
  }

  inline double mph_to_m_per_s(double mph) {
    return mph / 2.23694;
  }

  // Vehicle constants
  static constexpr double WHEEL_RADIUS_M = 0.041275;
  static constexpr double GEAR_RATIO = (16.0 / 36.0) * (18.0 / 36.0);
  static constexpr double VEHICLE_ACCEL_M_PER_S2 = 1;
  static constexpr int MAX_SPEED_MPH = 20;
  static constexpr double CURRENT_DRAW_WARN_THRESH = 5.0;
  static constexpr double CURRENT_DRAW_CRITICAL_THRESH = 36.0;
  static constexpr double BATTERY_VOLT_WARN_THRESH = 39.72;
  static constexpr double BATTERY_VOLT_CRITICAL_THRESH = 28.0;
  static constexpr double DISTANCE_WARN_FACTOR = 1.5;
  static constexpr double DISTANCE_CRITICAL_FACTOR = 1.1;
  static constexpr double DISTANCE_OFFSET_M = 2.0;

  static constexpr auto UPDATE_RATE = std::chrono::milliseconds(50);
  static constexpr auto HMI_TIMEOUT = std::chrono::milliseconds(1000);

  // Vehicle state variables
  double front_range_m = 0;
  double back_range_m = 0;

  bool vesc_fault = false;
  double speed_m_per_s = 0;
  double current_draw_a = 0;
  double battery_voltage = 0;

  bool hmi_forward = true;
  bool hmi_sensor_override = false;
  int hmi_goal_mph = 0;
  std::chrono::time_point<std::chrono::steady_clock> last_hmi_update;

  bool is_estopped = true;


  VelocityProfiler profiler;
  bool stopped_due_to_distance = false;
  bool stopped_due_to_loss_of_hmi = false;
  bool stopped_due_to_estop = false;

  inline void clear_stop_conditions() {
    stopped_due_to_distance = false;
    stopped_due_to_loss_of_hmi = false;
    stopped_due_to_estop = false;
  }

  inline uint8_t get_battery_state() {
    if (battery_voltage < BATTERY_VOLT_CRITICAL_THRESH) return hmi_msgs::msg::HmiInputState::CRITICAL;
    if (battery_voltage < BATTERY_VOLT_WARN_THRESH) return hmi_msgs::msg::HmiInputState::WARN;
    return hmi_msgs::msg::HmiInputState::OK;
  }

  inline uint8_t get_current_draw_state() {
    if (current_draw_a > CURRENT_DRAW_CRITICAL_THRESH) return hmi_msgs::msg::HmiInputState::CRITICAL;
    if (current_draw_a > CURRENT_DRAW_WARN_THRESH) return hmi_msgs::msg::HmiInputState::WARN;
    return hmi_msgs::msg::HmiInputState::OK;
  }

  inline uint8_t get_distance_state(double stopping_distance, double range) {
    if (range < stopping_distance * DISTANCE_CRITICAL_FACTOR) return hmi_msgs::msg::HmiInputState::CRITICAL;
    if (range < stopping_distance * DISTANCE_WARN_FACTOR) return hmi_msgs::msg::HmiInputState::WARN;
    return hmi_msgs::msg::HmiInputState::OK;
  }

};