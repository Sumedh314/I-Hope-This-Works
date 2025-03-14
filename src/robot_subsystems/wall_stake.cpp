#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void wall_stake_macro() {
    hood.set_zero_position(0);
    bool temp = false;

    while (true) {

        // Lower arm and pull up hood.
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }

        int start = pros::millis();
        while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (pros::millis() - start > 120) {
                temp = true;

                while (wall_stake.get_position() > -80) {
                    wall_stake.move(-127);
                    pros::delay(10);
                }
                wall_stake.brake();
                while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                    pros::delay(20);
                }
                while (wall_stake.get_position() < 0) {
                    wall_stake.move(127);
                    pros::delay(10);
                }
                wall_stake.brake();
                break;
            }
            pros::delay(20);
        }

        if (temp) {
            temp = false;
            continue;
        }

        while (wall_stake.get_position() < 100) {
            wall_stake.move(127);
            pros::delay(10);
        }
        wall_stake.brake();
        while (hood.get_position() < 450) {
            hood.move(127);
            pros::delay(10);
        }
        hood.brake();
        while (wall_stake.get_position() > -150) {
            wall_stake.move(-127);
            pros::delay(10);
        }
        wall_stake.brake();

        // Move arm up into position about to score ring.
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }
        while (wall_stake.get_position() < 450) {
            wall_stake.move(127);
            pros::delay(10);
        }
        wall_stake.brake();

        // Push ring onto stake.
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }
        while (wall_stake.get_position() < 650) {
            wall_stake.move(127);
            pros::delay(10);
        }
        wall_stake.brake();
        
        // Move hood back down.
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }
        while (hood.get_position() > 0) {
            hood.move(-127);
            pros::delay(10);
        }
        hood.brake();

        // Move arm back down.
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }
        while (wall_stake.get_position() > 0) {
            wall_stake.move(-127);
            pros::delay(10);
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

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            wall_stake.set_zero_position(0);
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