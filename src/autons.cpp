#include "main.h"
#include "autons.hpp"
#include "robot_subsystems/drive.hpp"
#include "robot_subsystems/intake.hpp"
#include "robot_subsystems/pneumatics.hpp"

void red_left() {

	// Set original pose of the robot.
	robot.set_coordinates(-14.5, -60);
	robot.set_original_heading(180);

	// Score preload on the allaince steak.
	robot.drive_distance(-15);
	robot.turn_to_heading(87, 1);
	robot.drive_distance(-3.7);
	pros::delay(300);
	intake_on();
	pros::delay(2500);
	intake_off();
	robot.set_drive_voltages(-127);
	pros::delay(200);
	robot.set_drive_voltages(127);
	pros::delay(200);
	robot.set_drive_voltages(-127);
	pros::delay(300);

	// Drive to goal and clamp it.
	robot.drive_distance(5);
	robot.drive_to_point(-19, -34, -1);
	pros::delay(500);
	robot.drive_distance(-12, 100);
	clamp_goal();
	pros::delay(500);

	// Score ring onto the goal.
	intake_on();
	robot.drive_to_point(-50, -24, 1);
	pros::delay(500);
	robot.drive_distance(-5);
	pros::delay(1000);

	// Score one of the rings near the autonomous line onto the goal.
	robot.drive_to_point(-53, -9);
	pros::delay(500);
	robot.drive_distance(-10);
	pros::delay(2000);

	// Score the other ring near the autonomous line onto the goal.
	robot.drive_distance(-10);
	robot.drive_to_point(-47, -9);
	pros::delay(500);
	robot.drive_distance(-5);
	pros::delay(2000);
	robot.drive_distance(-5);

	// Touch the ladder for the Autonomous Win Point.
	robot.drive_to_point(-24, -24);
	robot.drive_to_point(-3, -12, 1);
}

void red_right() {

	// Set original pose of the robot.
	robot.set_coordinates(14.5, -60);
	robot.set_original_heading(0);

	// Score preload on the alliance steak.
	robot.drive_distance(-15);
	robot.turn_to_heading(93, 1);
	robot.drive_distance(-3.7);
	pros::delay(300);
	intake_on();
	pros::delay(2500);
	intake_off();
	robot.set_drive_voltages(-127);
	pros::delay(200);
	robot.set_drive_voltages(127);
	pros::delay(200);
	robot.set_drive_voltages(-127);
	pros::delay(300);

	// Drive to goal and clamp it.
	robot.drive_distance(5);
	robot.drive_to_point(21, -34, -1);
	pros::delay(500);
	robot.drive_distance(-10, 100);
	clamp_goal();
	pros::delay(500);

	// Score ring onto the goal.
	intake_on();
	robot.drive_to_point(50, -24, 1);
	pros::delay(500);

	// Touch the ladder for the Autonomous Win Point.
	robot.drive_distance(-10);
	pros::delay(2000);
	intake_off();
	robot.drive_to_point(12, -10, 1);
}

void blue_left() {

	// Set original pose of the robot.
	robot.set_coordinates(-14.5, -60);
	robot.set_original_heading(180);

	// Score preload on the alliance steak.
	robot.drive_distance(-15);
	robot.turn_to_heading(87, 1);
	robot.drive_distance(-3.7);
	pros::delay(300);
	intake_on();
	pros::delay(2500);
	intake_off();
	robot.set_drive_voltages(-127);
	pros::delay(200);
	robot.set_drive_voltages(127);
	pros::delay(200);
	robot.set_drive_voltages(-127);
	pros::delay(300);

	// Drive to goal and clamp it.
	robot.drive_distance(5);
	robot.drive_to_point(-21, -34, -1);
	pros::delay(500);
	robot.drive_distance(-10, 100);
	clamp_goal();
	pros::delay(500);

	// Score ring onto the goal.
	intake_on();
	robot.drive_to_point(-50, -24, 1);
	pros::delay(500);

	// Touch the ladder for the Autonomous Win Point.
	robot.drive_distance(-10);
	pros::delay(2000);
	intake_off();
	robot.drive_to_point(-12, -10, 1);
}

void blue_right() {

	// Set original pose of the robot.
	robot.set_coordinates(14.5, -60);
	robot.set_original_heading(180);

	// Score preload on the allaince steak.
	robot.drive_distance(-15);
	robot.turn_to_heading(93, 1);
	robot.drive_distance(-3.7);
	pros::delay(300);
	intake_on();
	pros::delay(2500);
	intake_off();
	robot.set_drive_voltages(-127);
	pros::delay(200);
	robot.set_drive_voltages(127);
	pros::delay(200);
	robot.set_drive_voltages(-127);
	pros::delay(300);

	// Drive to goal and clamp it.
	robot.drive_distance(5);
	robot.drive_to_point(19, -34, -1);
	pros::delay(500);
	robot.drive_distance(-12, 100);
	clamp_goal();
	pros::delay(500);

	// Score ring onto the goal.
	intake_on();
	robot.drive_to_point(50, -24, 1);
	pros::delay(500);
	robot.drive_distance(-5);
	pros::delay(1000);

	// Score one of the rings near the autonomous line onto the goal.
	robot.drive_to_point(53, -9);
	pros::delay(500);
	robot.drive_distance(-10);
	pros::delay(2000);

	// Score the other ring near the autonomous line onto the goal.
	robot.drive_distance(-10);
	robot.drive_to_point(47, -9);
	pros::delay(500);
	robot.drive_distance(-5);
	pros::delay(2000);
	robot.drive_distance(-5);

	// Touch the ladder for the Autonomous Win Point.
	robot.drive_to_point(24, -24);
	robot.drive_to_point(3, -12, 1);
}

void skills_autonomous() {
	
	// Set original pose of the robot.
	robot.set_coordinates(0, -52);
	robot.set_original_heading(90);

	// Score preload on goal.
	intake_on();
	pros::delay(1000);

	// Stop intake and pick up goal.
	intake_off();
	robot.drive_to_point(0, -48);
	robot.turn_to_heading(0, -1);
	robot.drive_distance(-16);
	// clamp_goal();

	// Start intake and attempt to pick up another ring.
	intake_on();
	robot.turn_and_drive_to_point(24, -24, 1);

	// Pick up ring in the middle of the field.
	robot.turn_and_drive_to_point(0, 0);
	pros::delay(500);

	// Pick up ring next to robot.
	robot.drive_to_point(24, -24);
	robot.turn_and_drive_to_point(48, -24, 1, 1);

	// Pick up rings in the corner of the field.
	robot.turn_and_drive_to_point(48, -40, 1, 1);
	pros::delay(500);
	robot.drive_distance(7);
	robot.turn_and_drive_to_point(54, -48, 1, 1);

	// Put goal in the corner
	robot.turn_to_point(60, -60, -1);
	robot.drive_distance(-5);
	// unclamp_goal();
	intake_off();

	// Pick up other goal on this side of the field.
	robot.drive_distance(5);
	robot.turn_and_drive_to_point(-16, -48, -1, -1);
	// clamp_goal();

	// Score ring in front of the goal.
	intake_on();
	robot.turn_and_drive_to_point(-24, -24, 1, 1);

	// Score two more rings.
	robot.drive_to_point(-48, -24);
	robot.drive_to_point(-60, 0);

	// Score rings in the corner of the field.
	robot.drive_to_point(-48, -24);
	robot.turn_and_drive_to_point(-48, -40, 1, 1);
	pros::delay(500);
	robot.drive_distance(7);
	robot.turn_and_drive_to_point(-54, -48, 1, 1);

	// Put goal in the corner.
	robot.turn_to_point(-60, -60, -1);
	robot.drive_distance(-5);
	// unclamp_goal();
	intake_off();
}