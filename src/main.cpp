#include "main.h"
#include "robot-config.hpp"
#include "drive.hpp"
#include "intake.hpp"
#include "pneumatics.hpp"
#include "autons.hpp"

// Create PID and drivetrain objects used for the rest of the code.
PID drive_pid(20.5, 0, 1.7, 5, 1, 0.02, 4000, 0.01);
PID turn_pid(1.3, 0, 0, 15, 2, 0.2, 4000, 0.01);
Drive robot(
	3.25, 7, 0, 2.25, 36, 60, 2.8,
	front_left, middle_left, back_left, front_right, middle_right, back_right, inertial, vertical, horizontal,
	controller, drive_pid, turn_pid
);

// Auton number for auton selection
int auton_index = 0;
const char* autons[5] = {"Red left", "Red right", "Blue left", "Blue right", "Skills"};

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

	// Use the position of the intake to choose an autonomous routine.
	intake.set_zero_position(0);
	while (true) {
		auton_index = floor(intake.get_position() / 90.0);

		// Auton will just drive forward if no options are selected. Prints the choice to the bran and controller.
		if (auton_index > 4 || auton_index < 0) {
			pros::lcd::print(0, "Auton: drive forward 10 inches");
			controller.print(0, 0, "Auton: drive forward");
		}

		// Print the auton choice to the brain and controller.
		else {
			pros::lcd::print(0, "Auton: %s", autons[auton_index]);
			controller.print(0, 0, "Auton: %s", autons[auton_index]);
		}
		pros::delay(50);
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
	
	// Execute the correct autonomous routine based on what was chosen in the competition_initialize() function.
	switch (auton_index) {
		case 0:
			red_left();
			break;
		case 1:
			red_right();
			break;
		case 2:
			blue_left();
		case 3:
			blue_right();
			break;
		case 4:
			skills_autonomous();
			break;
		default:
			robot.drive_distance(10);
			break;
	}
}

/**
 * Continuously prints odometry information to the controller.
*/
void print_odom() {
	while (true) {
        controller.print(0, 0, "X coor: %f", robot.get_x());
        pros::delay(50);
        controller.print(1, 0, "Y coor: %f", robot.get_y());
        pros::delay(50);
        controller.print(2, 0, "Heading: %f", robot.get_heading());
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
	robot.set_original_heading(90);

	// Start tasks.
	pros::Task spin(spin_intake);
	pros::Task toggle(toggle_clamp);
	pros::Task drive([](){robot.split_arcade();});

	// Calibrate inertial sensor, wait for it to calibrate, and start odometry after a button on the controller is pressed.
	while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
		pros::delay(50);
	}
	inertial.reset();
	pros::delay(3000);
	pros::Task odom([](){robot.update_odometry();});
	pros::Task print(print_odom);
}