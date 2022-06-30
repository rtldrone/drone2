#include <webots/Robot.hpp>
#include <iostream>


int main() {
  webots::Robot robot;

  while (robot.step(32) != 1) {
    std::cout << "Running C++" << std::endl;
  }

  return 0;
}