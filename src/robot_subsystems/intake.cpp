#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void spin_intake() {
    while (true) {

		//high scorer stuff
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			hopper.move(-127);
			intake.move(127);
			redirect.move(-127);
		}
		//put balls in hopper
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			hopper.move(-127);
			intake.move(-127);
		}

		//mid tier scorer
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			hopper.move(-127);
			intake.move(127);
			redirect.move(127);
		}		

		//hopper in
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake.move(127);
			hopper.move(127);

		}

		else {
			intake.brake();
			redirect.brake();
			hopper.brake();
		}
		pros::delay(20);
	}
}

void intake_on(double voltage) {
	while (true){
		intake.move(voltage);
		hopper.move(voltage);
	}

}

void intake_off() {
	intake.brake();
	hopper.brake();
	redirect.brake();
}