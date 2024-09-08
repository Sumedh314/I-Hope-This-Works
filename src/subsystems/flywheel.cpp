#include "main.h"
#include "motion/pid.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/robot-config.hpp"

void spin_flywheel() {
    PID flywheel_pid(0.2, 0, 0, 100);

    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            flywheel.move(127);
            while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                pros::Task::delay(10);
            }
            while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                // std::uint32_t start = pros::millis();
                // // double voltage = flywheel_pid.compute(400 - flywheel.get_actual_velocity(), 50);
                // // flywheel.move(81 + voltage);
                // flywheel.move(127);
                // pros::Task::delay_until(&start, 10);
                pros::Task::delay(10);
            }
            flywheel.move(0);
            while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                pros::Task::delay(10);
            }
        }
        pros::Task::delay(20);
    }
}