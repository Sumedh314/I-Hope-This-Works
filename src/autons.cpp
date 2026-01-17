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
	robot.set_coordinates(13.5, -47.5);
	// Heading is already set to 90 in autonomous()

	intake_on();

	robot.drive_to_point(16, -33, 0, 60);
	pros::delay(1500);
	robot.drive_distance(4, 30);
	pros::delay(1000);
	robot.drive_distance(10, 30);
	pros::delay(1000);
	intake_off();

	robot.turn_to_point(0, 0);
	robot.drive_distance(15);
	midtier_on();
	pros::delay(10000);
	intake_off();
}

void red_right() {
	// not used
}

void blue_left() {
	// Set original pose of the robot
	// Heading is already set to 90 in autonomous()
	robot.set_coordinates(0, 0);

	pros::delay(100);
	//robot.drive_distance(2,30);

	robot.drive_to_point(0,23,1,30);
	pros::delay(1000);	
	intake_on();
	robot.turn_to_heading(180,1,30);
	robot.drive_distance(13, 20);
	pros::delay(1000);;
	robot.drive_distance(-4,30);
	robot.drive_distance(11, 30);
	pros::delay(1000);
	robot.turn_to_point(60, 115, 1); 
	intake_off();
                                                                                                                                                                                                                                                                                                                                     
	robot.drive_distance(14.5);
	midtier_on(60);
	pros::delay(10000);
	intake_off();
}

void skills_autonomous() {
	// Set original pose of the robot.
	robot.set_coordinates(0, -62.5);
	// Heading is already set to 90 in autonomous()

	// Score preload on goal.
	intake_on();
	pros::delay(1000);
	robot.set_drive_voltages(-127);
	pros::delay(200);
	robot.set_drive_voltages(127);
	pros::delay(100);
	robot.set_drive_voltages(-127);
	pros::delay(400);

	// Stop intake and pick up goal.
	intake_off();
	robot.drive_to_point(0, -48);
	robot.turn_and_drive_to_point(15, -48, -1);
	pros::delay(200);
	robot.drive_distance(-10, 100);
	pros::delay(200);
	deploy_loader();
	pros::delay(500);

	// Start intake and attempt to pick up another ring.
	intake_on();
	robot.turn_and_drive_to_point(24, -20, 1);
	pros::delay(600);

	// Pick up ring next to robot.
	robot.turn_and_drive_to_point(50, -24, 1, 1);
	pros::delay(600);

	// Pick up rings in the corner of the field.
	robot.turn_and_drive_to_point(48, -44, 1, 1);
	pros::delay(1500);
	robot.drive_distance(17);
	pros::delay(1500);
	robot.drive_distance(-22);
	robot.turn_to_heading(-45);
	robot.drive_distance(15);
	pros::delay(1500);

	// Put goal in the corner
	robot.turn_to_point(60, -60, -1);
	undeploy_loader();
	robot.drive_distance(-11);
	intake_off();

	// Pick up other goal on this side of the field.
	robot.drive_distance(9);
	robot.turn_and_drive_to_point(-13, -48, -1, -1);
	pros::delay(200);
	robot.drive_distance(-10, 100);
	pros::delay(200);
	deploy_loader();

	// Score ring in front of the goal.
	intake_on();
	robot.turn_and_drive_to_point(-24, -21, 1, 1);
	pros::delay(800);

	// Score two more rings.
	robot.drive_to_point(-50, -24, 1);
	pros::delay(800);
	robot.drive_to_point(-61, 6, 1);
	pros::delay(800);

	// Score rings in the corner of the field.
	robot.drive_to_point(-48, -25);
	robot.turn_and_drive_to_point(-48, -45, 1, 1);
	pros::delay(1500);
	robot.drive_distance(15);
	pros::delay(1500);
	robot.drive_distance(-22);
	robot.turn_to_heading(225);
	robot.drive_distance(15);
	pros::delay(1500);

	// Put goal in the corner.
	robot.turn_to_point(-60, -60, -1);
	robot.drive_distance(-10);
	undeploy_loader();
	intake_off();
	robot.turn_and_drive_to_point(-12, 60);
	robot.turn_to_point(-72, -72);
	robot.drive_to_point(-60, 60);
	robot.drive_distance(-10);
	robot.turn_to_heading(85);
	robot.drive_to_point(12, 60);
	robot.drive_to_point(60, 60);
	robot.drive_distance(-10);
}

void drive_ten_inches() {
	robot.drive_distance(10);
}