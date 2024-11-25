#include "main.h"
#include "autons.hpp"
#include "drive.hpp"
#include "intake.hpp"
#include "pneumatics.hpp"

void red_left() {

    // Robot starts facing right
	drive.set_original_heading(0);

	// score preload ring
	drive.drive_distance(16, 50);
	drive.turn_to_heading(90, 50);
	intake_on();
	drive.drive_distance(-4.2, 50);
	pros::delay(1000);
	intake_off();
	drive.drive_distance(5);
}

void red_right() {

    // Robot starts facing left
    drive.set_original_heading(180);

	// score preload ring
	drive.drive_distance(16, 50);
	drive.turn_to_heading(90, 50);
	intake_on();
	drive.drive_distance(-4.2, 50);
	pros::delay(1000);
	intake_off();
	drive.drive_distance(5);

	// align with mobile goal
	drive.turn_to_heading(235);
	drive.drive_distance(-33, 80);

	// pick up mobile goal
	clamp_goal();
	pros::delay(500);

	// pick up ring and score it
	drive.turn_to_heading(15);
	pros::delay(500);
	intake_on();
	drive.drive_distance(30);
}

void skills_autonomous() {

	//score preload on goal
	intake_on();
	pros::delay(1000);

	// stop intake and attempt to pick up clamp
	intake_off();
	drive.drive_distance(16);
	drive.turn_to_heading(180);
	drive.drive_distance(-16);
	clamp_goal();

	// start intake and attempt to pick up 2 rings
	drive.turn_to_heading(-90);
	intake_on();
	drive.drive_distance(48, 80);
	intake_off();

	// get in line with horizontal rings and attempt to score them
	drive.drive_distance(-12);
	drive.turn_to_heading(180);
	intake_on();
	drive.drive_distance(12);
	
	drive.turn_to_heading(0);
	drive.drive_distance(24);
}