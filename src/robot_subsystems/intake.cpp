#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void spin_intake() {
    while (true) {

		//move intake
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(-127);
		}

		//move intake other way
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(127);
		}



		else {
			intake.move(0);
		}
		pros::delay(20);
	}
}


void deploy_scorer() {
	while (true){
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			scoring_arm.move(-90);
			pros::delay(750);
			scoring_arm.move(90);
			pros::delay(750);
			scoring_arm.move(0);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			scoring_arm.move(90);
			pros::delay(500);
			scoring_arm.move(0);
		}
		else{
			scoring_arm.move(0);
			scoring_arm.brake();
		}
		pros::delay(50);

	}
}


void score(){
	scoring_arm.move(-127);
	pros::delay(1000);
	scoring_arm.move(90);
	pros::delay(500);
	scoring_arm.move(-127);
	pros::delay(1000);
	scoring_arm.move(90);
	pros::delay(500);
	scoring_arm.move(0);
}



void intake_on(double voltage) {
	intake.move(voltage);
	
}

void intake_off() {
	intake.brake();

}