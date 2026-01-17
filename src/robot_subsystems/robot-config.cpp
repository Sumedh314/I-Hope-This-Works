#include "main.h"
#include "robot_subsystems/robot-config.hpp"

// Motors
pros::Motor front_left(-18, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_left(-19, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_left(-20, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor front_right(13, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_right(12, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_right(14, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor intake(5, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor hopper(4, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor redirect(6, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);


// Sensors
pros::IMU inertial(1);
pros::adi::DigitalIn auton_select(6);
pros::Rotation vertical(2);
pros::Rotation horizontal(3);
// <<<<<<< HEAD
// pros::adi::Ultrasonic distance(7, 8);
// =======
// pros::adi::Ultrasonic distance(7, 8);
// >>>>>>> 2d648702d6b4fa138a9758c030fb603fd77a9afb

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut match_loader(8);