#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>

int main() {
    auto robot = std::make_shared<webots::Robot>();
    auto distance = robot->getDistanceSensor("asdf");
    distance->enable(32);

    while (robot->step(32) != 1) {
        std::cout << "Hello!" << std::endl;
    }

    return 0;
}