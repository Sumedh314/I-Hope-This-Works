#include "main.h"
#include "subsystems/robot-config.hpp"

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
		// intake.move((controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) - controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) * 80);
		pros::Task::delay(20);
	}
}