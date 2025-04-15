#include "main.h"
#include "robot_subsystems/robot-config.hpp"
#include "robot_subsystems/drive.hpp"
#include "robot_subsystems/intake.hpp"
#include "robot_subsystems/wall_stake.hpp"
#include "robot_subsystems/pneumatics.hpp"
#include "util.hpp"
#include "autons.hpp"

// Create PID and drivetrain objects used for the rest of the code.
PID drive_pid_IME(20.5, 0, 1.7, 10, 1, 1);
PID drive_pid(10, 0, 0.2, 5, 3, 3);
PID turn_pid(3, 1, 0.2, 15, 3, 3);
Drive robot(
	3.25, 7, 0.1, 2.25, 36, 60, 2.8,
	front_left, middle_left, back_left, front_right, middle_right, back_right, inertial, vertical, horizontal,
	controller, drive_pid_IME, drive_pid, turn_pid
);

// Auton number for auton selection and auton choices.
int auton_index = 0;
const char* autons[6] = {"red left", "red right", "blue left", "blue right", "skills", "drive 10 inches"};

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

	pros::lcd::register_btn1_cb(on_center_button);

	front_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	middle_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	back_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	front_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	middle_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	back_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	wall_stake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	hood.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

	// Make sure the robot isn't holding onto a goal at the end of the match.
	// unclamp_goal();
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

	// Uses the limit switch to choose an autonomous routine. This is better than having separate programs for each
	// autonomous routine because we don't have to manage multiple programs.
	while (true) {

		// Auton will do nothing if auton_index is equal to the length of autons.
		if (auton_index == sizeof(autons) / sizeof(const char*)) {
			pros::lcd::print(0, "Auton will do nothing.");
			controller.print(0, 0, "Auton: nothing");
		}

		// Choose one of the autons based on auton_index.
		else {
			pros::lcd::print(0, "Auton: %s", autons[auton_index]);
			controller.print(0, 0, "Auton: %s", autons[auton_index]);
		}

		// Wait for limit switch to be pressed.
		while (auton_select.get_value() == LOW) {
			pros::delay(50);
		}

		// Calibrate sensor if sensor is held pressed for 500 milliseconds.
		int start = pros::millis();
		while (auton_select.get_value() == HIGH) {
			if (pros::millis() - start > 500) {
				pros::lcd::print(0, "Calibrating IMU...");
				controller.print(0, 0, "Calibrating IMU...");

				inertial.reset(true);

				pros::lcd::print(0, "Done calibrating");
				controller.print(0, 0, "Done calibrating");
				pros::delay(1000);

				// Reduce because it will be increased after this loop while we want it to remain the same.
				auton_index--;
				
				// Wait for limit switch to be released.
				while (auton_select.get_value() == HIGH) {
					pros::delay(50);
				}
			}
			pros::delay(50);
		}

		// Increase auton_index by one and make it loop by using modulo.
		auton_index++;
		auton_index %= sizeof(autons) / sizeof(const char*) + 1;
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

	// Start odometry tasks.
	pros::Task odom([](){robot.update_odometry();});
	pros::Task print(print_odom);

	// Move wall stake mech up a little so the hood can move.
	wall_stake.move_relative(170, 100);

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
			break;
		case 3:
			blue_right();                                    
			break;
		case 4:
			skills_autonomous();
			break;
		case 5:
			drive_ten_inches();
			break;
		default:
			break;
	}
}

/**
 * Continuously prints odometry information to the controller.
*/
void print_odom() {
	while (true) {
        controller.print(2, 0, "(%0.2f, %0.2f)  ", robot.get_x(), robot.get_y());
        pros::delay(50);
        controller.print(2, 14, "   %0.2f°  ", robot.get_heading());
        pros::delay(50);
	}
}

/**
 * Vibrates the controller when there are 30 seconds left to alert the driver to not place goals in the positive corners.
*/
void dont_get_DQed() {
	int start = pros::millis();

	// Wait until 75000 milliseconds (1 minute and 15 seconds) have passed.
	while (pros::millis() - start <= 75000) {
		pros::delay(50);
	}
	controller.rumble("--");
}

/**
 * Ends the program when controller button LEFT is pressed.
*/
void e_stop() {
	while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			quick_exit(0);
		}
		pros::delay(50);
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
	pros::Task drive([](){robot.split_arcade();});
	pros::Task spin(spin_intake);
	pros::Task wall(wall_stake_macro);
	pros::Task toggle(toggle_clamp);
	pros::Task vibrate_controller(dont_get_DQed);

	int start = pros::millis();
	while (pros::millis() - start < 3000) {
		if (
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) ||
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) ||
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)
		) {
			if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
				auton_index = 0;
			}
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
				auton_index = 1;
			}
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
				auton_index = 2;
			}
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
				auton_index = 3;
			}
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
				auton_index = 4;
			}
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				auton_index = 6;
			}
			pros::Task stop(e_stop);

			drive.suspend();
			spin.suspend();
			toggle.suspend();

			controller.print(2, 0, "Loading...");
			inertial.reset();
			pros::delay(2000);
			robot.set_original_heading(90);

			autonomous();

			drive.resume();
			spin.resume();
			toggle.resume();

			break;
		}
		pros::delay(50);
	}

	pros::Task wall_manual(wall_stake_manual);
	pros::Task hood(move_hood_manual);
	pros::Task reset_odom([](){robot.reset_odometry();});
	pros::Task imu_odom([](){robot.IMU_odometry();});
}