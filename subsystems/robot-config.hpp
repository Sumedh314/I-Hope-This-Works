#include "main.h"

// Motors
extern pros::Motor front_left;
extern pros::Motor back_left;
extern pros::Motor middle_left;
extern pros::Motor front_right;
extern pros::Motor back_right;
extern pros::Motor middle_right;
extern pros::Motor intake;
extern pros::Motor flywheel;

// Sensors
extern pros::Imu inertial;
extern pros::ADIEncoder vertical_odom;
extern pros::ADIEncoder horizontal_odom;
extern pros::ADIUltrasonic auton_selector;

// Pneumatics
extern pros::ADIDigitalOut wings;
extern pros::ADIDigitalOut lift;

// Controller
extern pros::Controller controller;

// Drivetrain values
const double WHEEL_DIAMETER = 3.25;
const double TRACK_WIDTH = 12.5;
const double LEFT_OFFSET = 6.25;
const double BACK_OFFSET = 0;
const int MOTOR_GEAR_TEETH = 36;
const int WHEEL_GEAR_TEETH = 60;