#include "main.h"
#include "autons.hpp"
#include "robot_subsystems/drive.hpp"
#include "robot_subsystems/intake.hpp"
#include "robot_subsystems/pneumatics.hpp"
#include "robot_subsystems/robot-config.hpp"
void red_left() {

	// not used
}

void blue_right() {

	// Set original pose of the robot.
	//y, x
	robot.set_coordinates(13.5, -47.5);

	robot.set_original_heading(90);

	intake_on();

	robot.drive_to_point(16, -33, 0, 60);
	pros::delay(1500);
	robot.drive_distance(4, 30);
	pros::delay(1000);
	robot.drive_distance(10, 30);
	pros::delay(1000);
	intake_off();

	
	robot.turn_to_point(0,0);
	robot.drive_distance(15);
	midtier_on();
	pros::delay(10000);
	intake_off();
}

void red_right() {

	// not used
}

void blue_left() {

	// Set original pose of the robot.
	//y, x
	robot.set_coordinates(0, 0);
	robot.set_original_heading(90);

	// Score preload on the alliance stake.
	robot.drive_distance(2,127);
	pros::delay(100);
	intake_on();


	robot.drive_to_point(-19, -33, 0, 60);
	pros::delay(1500);
	robot.drive_distance(4, 30);
	pros::delay(1000);
	robot.drive_distance(9, 30);
	pros::delay(1000);
	intake_off();

	
	robot.turn_to_point(0,0);
	robot.drive_distance(15.5);
	midtier_on();
	pros::delay(10000);
	intake_off();

}

void skills_autonomous() {
	//skills auton

	

}

void drive_ten_inches() {
	robot.drive_distance(10);
}