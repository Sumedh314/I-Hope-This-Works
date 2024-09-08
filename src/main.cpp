#include "main.h"
#include "motion/odometry.hpp"
#include "subsystems/robot-config.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pneumatics.hpp"
#include "subsystems/flywheel.hpp"
#include "autons.hpp"

int auton = 1;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	inertial.reset(true);
	pros::Task::delay(100);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	intake.set_zero_position(0);
	while (true) {
		auton = floor(intake.get_position() / 90.0);
		if (auton == 1) {
			pros::lcd::print(2, "Auton: close");
			pros::Task::delay(50);
			pros::lcd::print(3, "Heading: %f", robot.get_heading());
			controller.print(0, 0, "Auton: close");
		}
		else if (auton == 2) {
			pros::lcd::print(2, "Auton: far");
			pros::Task::delay(50);
			pros::lcd::print(3, "Heading: %f", robot.get_heading());
			controller.print(0, 0, "Auton: far");
		}
		else if (auton == 3) {
			pros::lcd::print(2, "Auton: skills");
			pros::Task::delay(50);
			pros::lcd::print(3, "Heading: %f", robot.get_heading());
			controller.print(0, 0, "Auton: skills");
		}
		else {
			pros::lcd::print(2, "Auton: none");
			pros::Task::delay(50);
			pros::lcd::print(3, "Heading: %f", robot.get_heading());
			controller.print(0, 0, "Auton: none");
		}
		pros::Task::delay(50);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::Task odom(odometry);
	pros::Task::delay(100);
	
	if (auton == 1) {
		close();
	}
	else if (auton == 2) {
		far();
	}
	else if (auton == 3) {
		auton_skills();
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// Calibrate inertial sensor while blocking
	// inertial.set_rotation(180);

	// autonomous();
	// competition_initialize();

	int start = pros::millis();

    front_left.set_zero_position(0);

	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	flywheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	front_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	back_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	front_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	back_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	pros::Task drive(split_arcade);
	pros::Task pull(spin_intake);
	pros::Task odom(odometry);
	pros::Task fly(spin_flywheel);
	pros::Task wing(move_wings);
	pros::Task up(move_lift);
	pros::Task print(print_information);
	// close();
	// far();
	// auton_skills();

    // double path[4][2] = {{60, 90}, {59, 108}, {36, 132}, {24, 132}};
	// double path[14][2] = {{0, 0}, {6, 8}, {7, 13}, {8, 19}, {7, 25}, {4, 30}, {0, 32}, {-5, 32}, {-10, 28}, {-12, 22}, {-13, 15}, {-11, 8}, {-6, 3}, {0, 0}};
	// pros::Task::delay(100);
	// pure_pursuit(path, 14, 50, 1);

	// intake.move(127);
	// drive_for(12);
	// pros::Task::delay(500);
	// intake.brake();
	// pure_pursuit(path, 4, 30, -1);
	// drive_for(8);
	// turn_to_point(24, 132);
	// intake.move(-127);
	// pros::Task::delay(500);
	// drive_for(12);
	// drive_for(-12);
	// intake.brake();
	// turn_to_heading(270);
	// pros::Task pull(spin_intake);
	// pros::Task wing(move_wings);
	split_arcade();
}
