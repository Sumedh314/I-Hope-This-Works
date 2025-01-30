#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void spin_intake() {
    while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake_left.move(127);
			intake_right.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake_left.move(-127);
			intake_right.move(-127);
		}
		else {
			intake_left.brake();
			intake_right.brake();
		}
		pros::delay(20);
	}
}

void intake_on(double voltage) {
	intake_left.move(voltage);
	intake_right.move(voltage);
}

void intake_off() {
	intake_left.brake();
	intake_right.brake();
}