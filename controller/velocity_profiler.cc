#include "velocity_profiler.h"

#include <cmath>

VelocityProfiler::VelocityProfiler(double accel) : accel(accel) {}

void VelocityProfiler::set_goal(double v) { goal = v; }
double VelocityProfiler::get_goal() { return goal; }

void VelocityProfiler::update(double dt) {
  double dv = accel * dt;
  if (goal > setpoint_now) {
    setpoint_now += dv;
    // Make sure we don't overshoot the goal
    if (setpoint_now > goal) setpoint_now = goal;
  } else if (goal < setpoint_now) {
    setpoint_now -= dv;
    // Make sure we don't undershoot goal
    if (setpoint_now < goal) setpoint_now = goal;
  }
}

double VelocityProfiler::get_stopping_distance() {
  // Stopping distance is based on the current setpoint
  double stopping_time = std::abs(setpoint_now / accel);

  //dx = v0 * t + (1/2) * a * t^2
  return std::abs(setpoint_now) * stopping_time - 0.5 * accel * stopping_time * stopping_time;
}

double VelocityProfiler::get_setpoint() { return setpoint_now; }

void VelocityProfiler::reset() {
  goal = 0;
  setpoint_now = 0;
}