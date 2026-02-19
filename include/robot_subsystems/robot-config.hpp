#include "main.h"

// Motors
extern pros::Motor front_left;
extern pros::Motor middle_left;
extern pros::Motor front_right;
extern pros::Motor back_left;
extern pros::Motor middle_right;
extern pros::Motor back_right;

extern pros::Motor intake;
extern pros::Motor scoring_arm;




// Sensors
extern pros::IMU inertial;
extern pros::adi::DigitalIn auton_select;
extern pros::Rotation vertical;
extern pros::Rotation horizontal;
extern pros::adi::Ultrasonic distance;



//gps 
extern  pros::Gps gps1;
extern  pros::Gps gps2;



// Controller
extern pros::Controller controller;

// Pneumatics
extern pros::adi::DigitalOut match_loader;
extern pros::adi::DigitalOut chute;
extern pros::adi::DigitalOut descore;

//limit switch
extern pros::adi::DigitalIn limit_switch; 