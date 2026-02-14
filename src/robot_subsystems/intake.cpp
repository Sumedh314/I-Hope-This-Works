#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void spin_intake() {
    while (true) {

		//move intake
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake.move(127);
		}

		//move scoring arm
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			deploy_scorer();
		}

		//move scoring arm back
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			undeploy_scorer();
		}


		else {
			intake.move(0);
			scoring_arm.brake();
			intake.brake();
		}
		pros::delay(20);
	}
}


void deploy_scorer(){
    scoring_arm.move_relative(180,127);
	scoring_arm.move_relative(-180,127);

}

void undeploy_scorer(){
	scoring_arm.move_relative(-180,127);
}


void intake_on(double voltage) {
	intake.move(voltage);
	
}

void intake_off() {
	intake.brake();

}