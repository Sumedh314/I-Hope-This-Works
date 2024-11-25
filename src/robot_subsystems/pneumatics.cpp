#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void toggle_clamp() {
    bool clamped = false;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            clamp.set_value(!clamped);
            clamped = !clamped;
        }
        pros::delay(20);
    }
}

void clamp_goal() {
    clamp.set_value(HIGH);
}

void unclamp_goal() {
    clamp.set_value(LOW);
}