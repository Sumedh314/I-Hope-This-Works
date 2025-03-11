#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void wall_stake_macro() {
    while (true) {
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }

        wall_stake.move_relative(-35, 100);
        move_hood.move_relative(90, 100);

        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }

        wall_stake.move_relative(800, 100);
        
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::delay(20);
        }

        wall_stake.move_relative(50, 100);
        move_hood.move_relative(-90, 100);
        wall_stake.move_relative(-815, 100);

        pros::delay(20);
    }
}

void wall_stake_manual() {
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            wall_stake.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            wall_stake.move(-127);
        }
        else {
            wall_stake.brake();
        }
        pros::delay(20);
    }
}

void move_hood_manual() {
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            move_hood.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            move_hood.move(-127);
        }
        else {
            move_hood.brake();
        }
        pros::delay(20);
    }
}