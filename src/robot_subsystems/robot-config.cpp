#include "main.h"
#include "robot_subsystems/robot-config.hpp"

// Motors
pros::Motor front_left(-8, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_left(-9, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_left(-10, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor front_right(1, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_right(2, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_right(3, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor intake(17, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor hopper(19, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor redirect(18, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor hood(1, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees);

// Sensors
pros::IMU inertial(16);
pros::adi::DigitalIn auton_select(2); // UNCOMMENTED AND INITIALIZED
pros::Rotation vertical(-18);
pros::Rotation horizontal(4);
pros::adi::Ultrasonic distance(7, 8);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut goal_clamp(1);