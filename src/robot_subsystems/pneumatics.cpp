#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void toggle_loader() {
    bool deployed = false;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            match_loader.set_value(!deployed);
            deployed = !deployed;
        }
        pros::delay(20);
    }
}

void deploy_loader() {
    match_loader.set_value(HIGH);
}

void undeploy_loader() {
    match_loader.set_value(LOW);
}