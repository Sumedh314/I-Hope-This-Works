#include "main.h"
#include "drive.hpp"

// Motors
extern pros::Motor front_left;
extern pros::Motor middle_left;
extern pros::Motor front_right;
extern pros::Motor back_left;
extern pros::Motor middle_right;
extern pros::Motor back_right;

extern pros::Motor intake;

// Sensors
extern pros::IMU inertial;
extern pros::adi::Encoder vertical;
extern pros::adi::Encoder horizontal;

// Controller
extern pros::Controller controller;

// Pneumatics
extern pros::adi::DigitalOut clamp;