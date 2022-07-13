//
// Created by cameron on 7/12/22.
//

#include <iostream>
#include <gpiod.h>

int main() {
  struct gpiod_chip *chip1;
  struct gpiod_line *line1;

  //line1 = gpiod_line_find("gpio100");


  chip1 = gpiod_chip_open_by_name("gpiochip3");
  line1 = gpiod_chip_get_line(chip1, 4);

  gpiod_line_request_output(line1, "stacklight_node", false);
  gpiod_line_set_value(line1, true);

  gpiod_line_release(line1);
  gpiod_chip_close(chip1);
}