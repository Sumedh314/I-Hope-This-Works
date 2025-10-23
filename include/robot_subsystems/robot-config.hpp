#include "main.h"

// Motors
extern pros::Motor front_left;
extern pros::Motor middle_left;
extern pros::Motor front_right;
extern pros::Motor back_left;
extern pros::Motor middle_right;
extern pros::Motor back_right;

extern pros::Motor intake;
extern pros::Motor hopper;
extern pros::Motor redirect;




// Sensors
extern pros::IMU inertial;
extern pros::adi::DigitalIn auton_select;
extern pros::Rotation vertical;
extern pros::Rotation horizontal;
extern pros::adi::Ultrasonic distance;



// Controller
extern pros::Controller controller;

// Pneumatics
extern pros::adi::DigitalOut goal_clamp;