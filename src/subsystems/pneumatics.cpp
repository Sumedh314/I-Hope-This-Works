#include "main.h"
#include "subsystems/pneumatics.hpp"
#include "subsystems/robot-config.hpp"

void move_wings() {
    while (true) {
        wings.set_value(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
        pros::Task::delay(20);
    }
}

void move_lift() {
    bool lift_up = false;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.set_value(!lift_up);
            lift_up = !lift_up;
        }
        pros::Task::delay(20);
    }
}