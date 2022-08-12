#pragma once

class Blinker {
 public:
  enum Mode {
    OFF,
    ON,
    BLINK
  };

  Blinker(int num_ticks);
  void update();
  void set_mode(Mode mode);
  bool get_status();

 private:
  bool blink_state = false;
  int count = 0;
  int num_ticks = 0;
  Mode mode = OFF;
};