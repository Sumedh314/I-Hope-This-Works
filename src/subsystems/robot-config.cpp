#include "main.h"
#include "subsystems/robot-config.hpp"

// Motors
pros::Motor front_left(15, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor middle_left(20, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left(19, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right(18, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor middle_right(10, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right(9, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake(12, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor flywheel(1, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);

// Sensors
pros::Imu inertial(11);
pros::ADIEncoder vertical_odom(4, 5, false);
pros::ADIEncoder horizontal_odom(6, 7, false);
pros::ADIUltrasonic auton_selector(1, 2);

// Pneumatics
pros::ADIDigitalOut wings(2, LOW);
pros::ADIDigitalOut lift(3, LOW);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);