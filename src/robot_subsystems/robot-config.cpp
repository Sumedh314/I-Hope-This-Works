#include "main.h"
#include "robot_subsystems/robot-config.hpp"

// Motors
pros::Motor front_left(-3, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_left(-11, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_left(-9, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor front_right(5, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor middle_right(19, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);
pros::Motor back_right(6, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor intake(7, pros::v5::MotorGearset::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor wall_stake(21, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees);
pros::Motor hood(1, pros::v5::MotorGearset::green, pros::v5::MotorEncoderUnits::degrees);

// Sensors
pros::IMU inertial(16);
pros::adi::DigitalIn auton_select(2);
pros::Rotation vertical(-18);
pros::Rotation horizontal(4);
pros::adi::Ultrasonic distance(7, 8);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut goal_clamp(1);