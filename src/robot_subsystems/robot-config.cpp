#include "main.h"
#include "robot_subsystems/robot-config.hpp"

// Motors
pros::Motor front_left(-11, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_left(-12, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_left(-17, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor front_right(20, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_right(19, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_right(16, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor intake(1, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor scoring_arm(2, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees);


// Sensors
pros::IMU inertial(15);
pros::adi::DigitalIn auton_select(5);
pros::Rotation vertical(18);
pros::Rotation horizontal(21);

pros::Gps gps1(13);
pros::Gps gps2(3);

// pros::adi::Ultrasonic distance(7, 8);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut match_loader(8);
pros::adi::DigitalOut chute(6);
pros::adi::DigitalOut descore(7);
