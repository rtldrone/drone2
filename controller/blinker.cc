#include "blinker.h"

Blinker::Blinker(int num_ticks) : num_ticks(num_ticks) {}

bool Blinker::get_status() {
  switch (mode) {
    case OFF:
      return false;
    case ON:
      return true;
    case BLINK:
      return blink_state;
  }
}

void Blinker::update() {
  count++;
  if (count > num_ticks) {
    blink_state = !blink_state;
    count = 0;
  }
}

void Blinker::set_mode(Mode mode_) {
  mode = mode_;
}