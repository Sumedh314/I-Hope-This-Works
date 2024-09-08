#include "autons.hpp"
#include "motion/pid.hpp"
#include "motion/odometry.hpp"
#include "motion/pure_pursuit.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/robot-config.hpp"

void close() {
	// // First triball
    // intake.move(100);
	// drive_for(50);
	// intake.move(0);
	// turn_to_heading(0);
	// intake.move(-127);
	// pros::Task::delay(200);
	// drive_for(10);
	// pros::Task::delay(200);
	// intake.move(0);

	// // Second triball
	// drive_for(-5);
	// turn_to_heading(135);
	// intake.move(127);
	// drive_for(18);
	// intake.move(0);
	// turn_to_heading(0);
	// intake.move(-127);
	// pros::Task::delay(200);
	// drive_for(15);
	// intake.move(0);
	// pros::Task::delay(200);

	// // Third triball
	// drive_for(-10);
	// turn_to_heading(180);
	// intake.move(127);
	// drive_for(20);
	// pros::Task::delay(200);
	// intake.move(0);
	// turn_to_heading(0);
	// intake.move(-60);
	// pros::Task::delay(500);
	// drive_for(26);

	// // Fourth triball
	// intake.move(127);
	// drive_for(-10);
	// turn_to_heading(220);
	// drive_for(28);
	// pros::Task::delay(200);
	// drive_for(-28);
	// turn_to_heading(0);
	// intake.move(-127);
	// pros::Task::delay(500);
	// wings.set_value(HIGH);
	// drive_for(12);
	// drive_for(-10);
	// wings.set_value(LOW);
	// intake.brake();

	// Triball under elevation bar
	intake.move(127);
	drive_for(28);
	pros::Task::delay(200);
	flywheel.move(-60);
	drive_for(-28);
	intake.move(-30);
	turn_to_heading(270);
	intake.move(-127);
	pros::Task::delay(100);
	drive_for(8);
	flywheel.brake();

	// Get triball out of match load zone
	wings.set_value(HIGH);
	swing_left_to_heading(315);
	drive_for(15);
	wings.set_value(LOW);
	swing_left_to_heading(345);
	intake.move(-127);

	// Push in three triballs
	drive_for(6);
	drive_for(-8);
	turn_to_heading(160);
	drive_for(-20);

	// Fourth triball
	drive_for(6);
	swing_right_to_heading(65);
	intake.move(127);
	drive_for(43);
	intake.brake();

	// Outtake triball towards goal
	turn_to_heading(300);
	intake.move(-127);
	pros::Task::delay(500);

	// Fifth triball
	turn_to_heading(25);
	intake.move(127);
	drive_for(20);

	// Push in remaining triballs
	turn_to_heading(270);
	intake.move(-127);
	pros::Task::delay(500);
	wings.set_value(HIGH);
	drive_for(35);
	wings.set_value(LOW);
	intake.brake();
	drive_for(-10);
}

void far() {
	robot.set_x(55);
	robot.set_y(30);
	robot.set_heading(90);

	// Push preload into goal
	turn_to_heading(50);
	drive_for(-18);
	swing_left_to_heading(0);
	drive_for(-6.5);

	// Get triball out of match load zone
	drive_for(8.5);
	swing_left_to_heading(40);
	wings.set_value(HIGH);
	drive_for(14);

	// Touch elevation bar
	swing_left_to_heading(85);
	wings.set_value(LOW);
	drive_for(32);

	// turn_to_heading(100);
	// drive_for(9.5);
	// robot.set_x(60);
	// robot.set_y(30);
	// // robot.set_heading(270);

    // turn_to_heading(45);
	// wings.set_value(HIGH);
	// drive_for(18);
	// wings.set_value(LOW);
	// turn_to_heading(0);
	// intake.move(-127);
	// pros::Task::delay(500);
	// drive_for(10);
	// drive_for(-8);
	// intake.brake();
	// turn_to_heading(45);
	// drive_for(-24);
	// turn_to_heading(270);
	// drive_for(24);
	// turn_to_heading(280);
	// drive_for(18);
	// turn_to_heading(0);
	// intake.move(-127);
	// drive_for(12);
}

void auton_skills() {
	robot.set_x(55);
	robot.set_y(30);
	robot.set_heading(90);
	turn_to_heading(50);
	drive_for(-20);
	turn_to_heading(0);
	drive_for(-12);
	drive_for(16);
	turn_to_heading(100);
	drive_for(-3);
	lift.set_value(HIGH);
	flywheel.move(127);
	uint32_t start = pros::millis();
	pros::Task::delay_until(&start, 30000);
	lift.set_value(LOW);
	flywheel.move(0);
	intake.move(-127);

	turn_to_heading(225);
	drive_for(-20);
	turn_to_heading(270);
	drive_for(-79);
	swing_right_to_heading(315);
	drive_for(-18);
	swing_right_to_heading(0);
	drive_for(-12);
	drive_for(12);
	drive_for(-12);
	drive_for(6);
	turn_to_heading(270);
	drive_for(48);
	turn_to_heading(180);
	drive_for(36);
	turn_to_heading(90);
	wings.set_value(HIGH);
	drive_for(30);
	wings.set_value(LOW);
	drive_for(-30);

	// turn_to_heading(50);
	// double path[6][2] = {{40, 18}, {53, 30}, {62, 70}, {60, 100}, {40, 125}, {25, 135}};
	// pure_pursuit(path, 6, 100, 1);
	// drive_for(-12);
	// turn_to_heading(90);
	// double path2[3][2] = {{35, 135}, {25, 100}, {-24, 80}};
	// pure_pursuit(path2, 3, 40, -1);
	// turn_to_heading(90);
	// wings.set_value(HIGH);
	// drive_for(24);
	// drive_for(-24);
	// wings.set_value(LOW);

	// turn_to_heading(70);
	// drive_for(20);
	// turn_to_heading(90);
	// drive_for(72);
	// turn_to_heading(135);
	// drive_for(32);
	// turn_to_heading(180);
	// drive_for(9);
	// drive_for(-9);
	// turn_to_heading(260);
	// drive_for(50);
	// turn_to_heading(180);
	// drive_for(36);
	// turn_to_heading(90);
	// wings.set_value(HIGH);
	// drive_for(24);
	// drive_for(-24);
}