#include "main.h"

// Motors
extern pros::Motor front_left;
extern pros::Motor middle_left;
extern pros::Motor front_right;
extern pros::Motor back_left;
extern pros::Motor middle_right;
extern pros::Motor back_right;

extern pros::Motor intake;

extern pros::Motor wall_stake;
extern pros::Motor hood;

// Sensors
extern pros::IMU inertial;
extern pros::adi::DigitalIn auton_select;
extern pros::adi::Encoder vertical;
extern pros::adi::Encoder horizontal;

// Controller
extern pros::Controller controller;

// Pneumatics
extern pros::adi::DigitalOut goal_clamp;