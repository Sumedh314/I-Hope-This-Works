#include "main.h"
#include "autons.hpp"
#include "robot_subsystems/drive.hpp"
#include "robot_subsystems/intake.hpp"
#include "robot_subsystems/pneumatics.hpp"

void red_left() {
    // // Robot starts facing left
    // robot.set_original_heading(180);

	// // score preload ring
	// robot.drive_distance(16, 50);
	// robot.turn_to_heading(90, 50);
	// intake_on();
	// robot.drive_distance(-4.2, 50);
	// pros::delay(1000);
	// intake_off();
	// robot.drive_distance(5);

	// // align with mobile goal
	// robot.turn_to_heading(235);
	// robot.drive_distance(-33, 80);

	// // pick up mobile goal
	// clamp_goal();
	// pros::delay(500);

	// // pick up ring and score it
	// robot.turn_to_heading(15);
	// pros::delay(500);
	// intake_on();
	// robot.drive_distance(30);

    // // Robot starts facing right
	// robot.set_original_heading(0);

	// // score preload ring
	// robot.drive_distance(16, 50);
	// robot.turn_to_heading(90, 50);
	// intake_on();
	// robot.drive_distance(-4.2, 50);
	// pros::delay(1000);
	// intake_off();
	// robot.drive_distance(5);

	robot.set_coordinates(-14.5, -60);
	robot.set_original_heading(180);
	pros::delay(100);
	pros::Task odom([](){robot.update_odometry();});
	pros::delay(100);
	pros::Task print(print_odom);

	robot.drive_distance(-15);
	robot.turn_to_heading(87);
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
	robot.drive_distance(5);
	robot.set_drive_voltages(127, -127);
	pros::delay(300);
	robot.drive_to_point(-19, -34);
	pros::delay(500);
	robot.drive_distance(-12, 100);
	clamp_goal();
	pros::delay(500);
	robot.set_drive_voltages(127, -127);
	intake_on();
	pros::delay(300);
	robot.drive_to_point(-50, -24);
	pros::delay(500);
	robot.drive_distance(-5);
	pros::delay(1000);
	robot.drive_to_point(-53, -9);
	pros::delay(500);
	robot.drive_distance(-10);
	pros::delay(2000);
	robot.drive_distance(-10);
	robot.drive_to_point(-47, -9);
	pros::delay(500);
	robot.drive_distance(-5);
	pros::delay(2000);
	robot.drive_distance(-5);
	robot.drive_to_point(-24, -24);
	robot.set_drive_voltages(127, -127);
	pros::delay(300);
	robot.drive_to_point(-3, -12);
}

void red_right() {
	robot.set_coordinates(14.5, -60);
	robot.set_original_heading(0);
	pros::delay(100);
	pros::Task odom([](){robot.update_odometry();});
	pros::delay(100);
	pros::Task print(print_odom);

	robot.drive_distance(-15);
	robot.turn_to_heading(93);
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
	robot.drive_distance(5);
	robot.set_drive_voltages(-127, 127);
	pros::delay(300);
	robot.drive_to_point(21, -34);
	pros::delay(500);
	robot.drive_distance(-10, 100);
	clamp_goal();
	pros::delay(500);
	robot.set_drive_voltages(-127, 127);
	intake_on();
	pros::delay(300);
	robot.drive_to_point(50, -24);
	pros::delay(500);
	robot.drive_distance(-10);
	pros::delay(2000);
	intake_off();
	robot.set_drive_voltages(-127, 127);
	pros::delay(300);
	robot.drive_to_point(12, -10);
}

void blue_left() {

}

void blue_right() {
	
}

void skills_autonomous() {

	//score preload on goal
	intake_on();
	pros::delay(1000);

	// stop intake and attempt to pick up clamp
	intake_off();
	robot.drive_distance(16);
	robot.turn_to_heading(180);
	robot.drive_distance(-16);
	clamp_goal();

	// start intake and attempt to pick up 2 rings
	robot.turn_to_heading(-90);
	intake_on();
	robot.drive_distance(48, 80);
	intake_off();

	// get in line with horizontal rings and attempt to score them
	robot.drive_distance(-12);
	robot.turn_to_heading(180);
	intake_on();
	robot.drive_distance(12);
	
	robot.turn_to_heading(0);
	robot.drive_distance(24);
}