#pragma once

class VelocityProfiler {
 public:
  VelocityProfiler(double accel);

  void set_goal(double v);
  double get_goal();
  void update(double dt);
  double get_setpoint();
  double get_stopping_distance();
  void reset();

 private:
  const double accel;
  double goal = 0;
  double setpoint_now = 0;
};