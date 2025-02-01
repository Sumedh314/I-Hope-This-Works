#include "main.h"
#include "robot_subsystems/robot-config.hpp"

// Motors
pros::Motor front_left(-11, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_left(-12, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_left(-13, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor front_right(17, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_right(19, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_right(20, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor intake_left(-18, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor intake_right(21, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

// Sensors
pros::IMU inertial(16);
pros::adi::DigitalIn auton_select(2);
pros::adi::Encoder vertical(5, 6, false);
pros::adi::Encoder horizontal(3, 4, false);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut goal_clamp(1);