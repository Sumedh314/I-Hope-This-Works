#include "main.h"
#include "autons.hpp"
#include "robot_subsystems/drive.hpp"
#include "robot_subsystems/intake.hpp"
#include "robot_subsystems/pneumatics.hpp"
#include "robot_subsystems/robot-config.hpp"

void alliance_stake() {
	robot.turn_to_heading(90, 1);
	robot.drive_distance(-3.25);
	pros::delay(300);
	intake_on();
	pros::delay(1000);
	robot.set_drive_voltages(-70);
	pros::delay(200);
	robot.set_drive_voltages(70);
	pros::delay(110);
	robot.set_drive_voltages(-70);
	pros::delay(400);
	intake_off();
}

void red_left() {

	// Set original pose of the robot.
	robot.set_coordinates(-11.5, -62.5);
	robot.set_original_heading(180);

	pros::delay(1000);

	// Score preload on the allaince stake.
	robot.drive_distance(-16);
	alliance_stake();

	// Drive to goal and clamp it.
	robot.drive_distance(5);
	robot.drive_to_point(-14, -37, -1);
	pros::delay(100);
	robot.drive_distance(-10, 100);
	clamp_goal();
	pros::delay(150);

	// Score ring onto the goal.
	intake_on();
	robot.drive_to_point(-45, -29, 1);
	pros::delay(200);
	robot.drive_distance(-5);
	pros::delay(300);

	// Score one of the rings near the autonomous line onto the goal.
	robot.drive_to_point(-41, -13, 1);
	pros::delay(100);
	robot.drive_distance(-10);
	pros::delay(200);

	// Score the other ring near the autonomous line onto the goal.
	// double path[5][2] = {{robot.get_x(), robot.get_y()}, {-45, -20}};
	// robot.follow_path(path, 2, 60, -1);
	robot.set_drive_voltages(-127, 127);
	pros::delay(150);
	robot.drive_to_point(-47, -24, -1);
	robot.drive_to_point(-48, -13, 1);
	pros::delay(100);
	robot.drive_distance(-5);
	pros::delay(200);
	// robot.drive_distance(-5);

	// Touch the ladder for the Autonomous Win Point.
	wall_stake.move_relative(650, 100);
	robot.drive_to_point(-24, -30);
	robot.drive_to_point(-15, -19, 1);
}

void blue_right() {

	// Set original pose of the robot.
	robot.set_coordinates(11.5, -62.5);
	robot.set_original_heading(0);

	// Score preload on the allaince stake.
	robot.drive_distance(-16);
	alliance_stake();

	// Drive to goal and clamp it.
	robot.drive_distance(5);
	robot.drive_to_point(14, -37, -1);
	pros::delay(100);
	robot.drive_distance(-10, 100);
	clamp_goal();
	pros::delay(150);

	// Score ring onto the goal.
	intake_on();
	robot.drive_to_point(45, -29, 1);
	pros::delay(100);
	robot.drive_distance(-5);
	pros::delay(200);

	// Score one of the rings near the autonomous line onto the goal.
	robot.drive_to_point(41, -13, 1);
	pros::delay(100);
	robot.drive_distance(-10);
	pros::delay(200);

	// Score the other ring near the autonomous line onto the goal.
	robot.drive_to_point(46, -24, -1);
	robot.drive_to_point(48, -13, 1);
	pros::delay(100);
	robot.drive_distance(-5);
	pros::delay(200);

	// Touch the ladder for the Autonomous Win Point.
	wall_stake.move_relative(650, 100);
	robot.drive_to_point(24, -30);
	robot.drive_to_point(15, -19, 1);
}

void red_right() {

	// Set original pose of the robot.
	robot.set_coordinates(11.5, -62.5);
	robot.set_original_heading(0);

	// Score preload on the alliance stake.
	robot.drive_distance(-16);
	alliance_stake();

	// Drive to goal and clamp it.
	robot.drive_distance(5);
	robot.drive_to_point(14, -37, -1);
	pros::delay(100);
	robot.drive_distance(-10, 100);
	clamp_goal();
	pros::delay(150);

	// Score ring onto the goal.
	intake_on();
	robot.drive_to_point(45, -27, 1);
	pros::delay(1000);
	intake_off();
	// while (robot.get_heading() > -60) {
	// 	robot.set_drive_voltages(127, 0);
	// }
	// robot.brake();

	// Score ring in the corner
	// robot.drive_distance(-10);
	robot.drive_to_point(50, -40);
	intake_on();
	robot.set_drive_voltages(127);
	pros::delay(1200);

	// Touch the ladder for the Autonomous Win Point.
	robot.drive_distance(-10);
	pros::delay(200);
	wall_stake.move_relative(650, 100);
	robot.drive_to_point(19, -20, 1);
}

void blue_left() {

	// Set original pose of the robot.
	robot.set_coordinates(-11.5, -62.5);
	robot.set_original_heading(180);

	// Score preload on the alliance stake.
	robot.drive_distance(-16);
	alliance_stake();

	// Drive to goal and clamp it.
	robot.drive_distance(5);
	robot.drive_to_point(-14, -37, -1);
	pros::delay(100);
	robot.drive_distance(-10, 100);
	clamp_goal();
	pros::delay(150);

	// Score ring onto the goal.
	intake_on();
	robot.drive_to_point(-45, -27, 1);
	pros::delay(500);

	// Score ring in the corner
	// robot.drive_distance(-10);
	intake_off();
	robot.drive_to_point(-45, -40);
	intake_on();
	robot.set_drive_voltages(127);
	pros::delay(700);

	// Touch the ladder for the Autonomous Win Point.
	robot.drive_distance(-10);
	pros::delay(200);
	wall_stake.move_relative(650, 100);
	robot.drive_to_point(-19, -20, 1);
}

void skills_autonomous() {
	
	// Set original pose of the robot.
	robot.set_coordinates(0, -62.5);
	robot.set_original_heading(90);

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
	clamp_goal();
	pros::delay(500);

	// Start intake and attempt to pick up another ring.
	intake_on();
	robot.turn_and_drive_to_point(24, -20, 1);
	pros::delay(600);

	// // Pick up ring in the middle of the field.
	// robot.turn_and_drive_to_point(0, 0);
	// pros::delay(500);

	// Pick up ring next to robot.
	// robot.drive_to_point(48, -24);
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
	unclamp_goal();
	robot.drive_distance(-11);
	intake_off();

	// Pick up other goal on this side of the field.
	robot.drive_distance(9);
	robot.turn_and_drive_to_point(-13, -48, -1, -1);
	pros::delay(200);
	robot.drive_distance(-10, 100);
	pros::delay(200);
	clamp_goal();
	pros::delay(500);

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
	unclamp_goal();
	intake_off();
	wall_stake.move_relative(-70, 200);
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
	wall_stake.move_relative(800, 200);
}