#include "main.h"
#include "robot-config.hpp"
#include "drive.hpp"
#include "intake.hpp"
#include "pneumatics.hpp"
#include "autons.hpp"

// Create PID and drivetrain objects used for the rest of the code.
PID drive_pid(20.5, 0, 1.7, 5, 1, 0.02, 4000, 0.01);
PID turn_pid(1.6, 0, 0, 15, 2, 0.2, 4000, 0.01);
Drive drive(
	3.25, 7, 0, 2.25, 36, 60, 2.8,
	front_left, middle_left, back_left, front_right, middle_right, back_right, inertial, vertical, horizontal,
	drive_pid, turn_pid, controller
);

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

	front_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	middle_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	back_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	front_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	middle_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	back_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	inertial.reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	unclamp_goal();
}

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
	int auton = 0;
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
	// drive_for(15);
	red_left();
	// skills_autonomous();	
}

/**
 * Continuously prints odometry information to the controller.
*/
void print_odom() {
	while (true) {
        controller.print(0, 0, "X coor: %f", drive.get_x());
        pros::delay(50);
        controller.print(1, 0, "Y coor: %f", drive.get_y());
        pros::delay(50);
        controller.print(2, 0, "Heading: %f", drive.get_heading());
        pros::delay(50);
	}
}

/**
 * Update odometry. Using pros::Task odom(drive.update_odometry); doesn't work for some reason.
*/
// void odometry() {
// 	drive.update_odometry();
// }

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
	// The robot starts facing 90 degrees (forward).
	drive.set_original_heading(90);

	// Calibrate inertial sensor and wait for it to calibrate.
	inertial.reset();
	pros::delay(3000);

	// Start tasks.
	pros::Task odom([](){drive.update_odometry();});
	pros::Task spin(spin_intake);
	pros::Task toggle(toggle_clamp);
	pros::Task print(print_odom);

	// Drive the robot from the controller using split arcade.
	drive.split_arcade();
}