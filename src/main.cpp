#include "main.h"
#include "robot_subsystems/robot-config.hpp"
#include "robot_subsystems/drive.hpp"
#include "robot_subsystems/intake.hpp"
#include "robot_subsystems/pneumatics.hpp"
#include "util.hpp"
#include "autons.hpp"

// Create PID and drivetrain objects used for the rest of the code.
PID drive_pid_IME(20.5, 0, 1.7, 10, 1, 1);
PID drive_pid(10, 0, 0.2, 5, 3, 3);
PID turn_pid(3, 1, 0.2, 15, 3, 3);
Drive robot(
	3.25, 7, 0.1, 2.25, 36, 60, 2.8,
	front_left, middle_left, back_left, front_right, middle_right, back_right,
	controller, drive_pid_IME, drive_pid, turn_pid
);

bool auton_happened = false;

// Auton number for auton selection and auton choices.
int auton_index = 0;
const char* autons[6] = {"red left", "red right", "blue left", "blue right", "skills", "drive 10 inches"};

/**
 * LCD button callback
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
 * Runs initialization code.
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
	hood.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Disabled
 */
void disabled() {
	// unclamp_goal();
}

/**
 * Competition initialization (auton selector)
 */
void competition_initialize() {
	while (true) {
		if (auton_index == sizeof(autons) / sizeof(const char*)) {
			pros::lcd::print(0, "Auton will do nothing.");
			controller.print(0, 0, "Auton: nothing");
		} else {
			pros::lcd::print(0, "Auton: %s", autons[auton_index]);
			controller.print(0, 0, "Auton: %s", autons[auton_index]);
		}

		while (auton_select.get_value() == LOW) {
			pros::delay(50);
		}

		// Wait for limit switch release
		while (auton_select.get_value() == HIGH) {
			pros::delay(50);
		}

		auton_index++;
		auton_index %= sizeof(autons) / sizeof(const char*) + 1;
		pros::delay(50);
	}
}

/**
 * Autonomous
 */
void autonomous() {
	switch (auton_index) {
		case 0: red_left(); break;
		case 1: red_right(); break;
		case 2: blue_left(); break;
		case 3: blue_right(); break;
		case 4: skills_autonomous(); break;
		case 5: drive_ten_inches(); break;
		default: break;
	}

	auton_happened = true;
}

/**
 * 30s warning vibration
 */
void dont_get_DQed() {
	int start = pros::millis();
	while (pros::millis() - start <= 75000) {
		pros::delay(50);
	}
	// controller.rumble("--");
}

/**
 * Emergency stop
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
 * Operator Control
 */
void opcontrol() {
	pros::Task drive([](){robot.split_arcade();});
	pros::Task spin(spin_intake);
	pros::Task toggle(toggle_clamp);
	pros::Task vibrate_controller(dont_get_DQed);

	int start = pros::millis();

	while (pros::millis() - start < 2100) {
		if (
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) ||
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) ||
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)
		) {
			if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) auton_index = 0;
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) auton_index = 1;
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) auton_index = 2;
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) auton_index = 3;
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) auton_index = 4;
			else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) auton_index = 6;

			pros::Task stop(e_stop);

			drive.suspend();
			spin.suspend();
			toggle.suspend();

			controller.print(2, 0, "Loading...");
			pros::delay(2100);

			autonomous();

			drive.resume();
			spin.resume();
			toggle.resume();
			break;
		}
		pros::delay(50);
	}
}
