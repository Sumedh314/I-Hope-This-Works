#include "main.h"
#include "autons.hpp"
#include "robot_subsystems/drive.hpp"
#include "robot_subsystems/intake.hpp"
#include "robot_subsystems/pneumatics.hpp"
#include "robot_subsystems/robot-config.hpp"

void red_left() {
	robot.set_original_heading(-90);

}

void gps_reset() {
	double x = 0;
	double y = 0;
	double heading = 0;

	robot.get_averaged_gps_position(x,y,heading);
	robot.set_coordinates(x,y);
}

void check_gps(){
	double coor1 = 0;
	double coor2 = 0;
	double coor3 = 0;
	robot.get_averaged_gps_position(coor1, coor2, coor3);
}

void blue_right() {
	pros::delay(100);
	robot.drive_distance(5,50);
	pros::delay(100);
	check_gps();
}

void red_right() {

	//not used
}

void blue_left() {
	// pros::delay(100);
	// robot.turn_and_drive_to_point(-15,30);
	// pros::delay(100);
	// check_gps();

	pros::delay(100);

	robot.drive_distance(26,40);
	pros::delay(100);
	intake_on();
	robot.turn_to_heading(0,1,30);
	pros::delay(100);
	robot.drive_distance(13, 15);
	pros::delay(1000);
	robot.drive_distance(-4,40);
	robot.turn_to_heading(170);
	pros::delay(200);
	robot.drive_distance(11, 20);
	pros::delay(200);
	robot.turn_to_heading(40,0);
	pros::delay(200);
	pros::delay(200);
	robot.drive_distance(-16);
	intake_off();
	pros::delay(200);
	score();

}

void skills_autonomous() {
	

	//first blocks
	pros::delay(100);
	robot.drive_distance(25, 40);
	pros::delay(100);

	//second blocks
	robot.turn_to_heading(180,1,40);
	pros::delay(100);
	robot.drive_distance(15,40);
	pros::delay(100);

	//third blocks
	robot.turn_to_heading(90,1,40);
	pros::delay(100);
	robot.drive_distance(45,40);
	pros::delay(100);

	robot.turn_to_heading(0,0,40);
	pros::delay(100);
	robot.drive_distance(45,40);
	pros::delay(100);

	//score high goal
	robot.drive_distance(13,40);
	pros::delay(100);
	robot.turn_to_heading(-90,0,40);


	intake_off();
	pros::delay(200);
	score();
}

void drive_ten_inches() {
	robot.drive_distance(10);
}