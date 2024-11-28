#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void spin_intake() {
    while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(-127);
		}
		else {
			intake.brake();
		}
		pros::delay(20);
	}
}

void intake_on(double voltage) {
	intake.move(voltage);
}

void intake_off() {
	intake.brake();
}