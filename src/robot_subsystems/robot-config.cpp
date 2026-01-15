#include "main.h"
#include "robot_subsystems/robot-config.hpp"

// Motors
pros::Motor front_left(-13, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_left(-12, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_left(-11, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor front_right(18, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_right(19, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_right(20, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor intake(1, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor hopper(2, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor redirect(3, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);


// Sensors
pros::IMU inertial(17);
pros::adi::DigitalIn auton_select(2);
pros::Rotation vertical(15);
pros::Rotation horizontal(14);
// <<<<<<< HEAD
pros::adi::Ultrasonic distance(7, 8);
// =======
// pros::adi::Ultrasonic distance(7, 8);
// >>>>>>> 2d648702d6b4fa138a9758c030fb603fd77a9afb

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut match_loader(8);