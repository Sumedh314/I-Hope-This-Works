#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void wall_stake_macro() {
    wall_stake.set_zero_position(0);
    hood.set_zero_position(0);

    while (true) {
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }

        while (wall_stake.get_position() < 100) {
            wall_stake.move(127);
        }
        wall_stake.brake();
        while (hood.get_position() < 400) {
            hood.move(127);
        }
        hood.brake();
        while (wall_stake.get_position() > -150) {
            wall_stake.move(-127);
        }
        wall_stake.brake();

        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }

        while (wall_stake.get_position() < 450) {
            wall_stake.move(127);
        }
        wall_stake.brake();

        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }

        while (wall_stake.get_position() < 650) {
            wall_stake.move(127);
        }
        wall_stake.brake();
        
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }
        while (hood.get_position() > 0) {
            hood.move(-127);
        }
        hood.brake();

        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }

        while (wall_stake.get_position() > 0) {
            wall_stake.move(-127);
        }
        wall_stake.brake();

        pros::delay(20);
    }
}

void wall_stake_manual() {
    while (true) {
        if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                wall_stake.move(127);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                wall_stake.move(-127);
            }
            else {
                wall_stake.brake();
            }
        }
        pros::delay(20);
    }
}

void move_hood_manual() {
    while (true) {
        if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                hood.move(127);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                hood.move(-127);
            }
            else {
                hood.brake();
            }
        }
        pros::delay(20);
    }
}