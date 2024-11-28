#ifndef DRIVE_HPP
#define DRIVE_HPP
#include "main.h"
#include "pid.hpp"
#include "point.hpp"

class Drive : public Point{
    
    // Constants related to the drivetrain. Distances are in inches.
    const double DRIVE_WHEEL_DIAMETER;
    const double TRACK_WIDTH;
    const double LEFT_OFFSET;
    const double HORIZONTAL_OFFSET;
    const int MOTOR_GEAR_TEETH;
    const int WHEEL_GEAR_TEETH;
    const double TRACKING_WHEEL_DIAMETER;
    
    // The heading the robot starts at in the beginning of the match.
    double original_heading;

    // The hardware used in this drivetrain.
    // I have no idea if this will work or not. ChatGPT said to put the & symbol to pass by reference.
    pros::Motor& front_left;
    pros::Motor& middle_left;
    pros::Motor& front_right;
    pros::Motor& back_left;
    pros::Motor& middle_right;
    pros::Motor& back_right;

    pros::IMU& inertial;
    pros::adi::Encoder& vertical;
    pros::adi::Encoder& horizontal;

    pros::Controller& controller;

    // PID objects used for this drivetrain.
    PID drive_pid;
    PID turn_pid;

    public:
        Drive(
            const double DRIVE_WHEEL_DIAMETER, const double TRACK_WIDTH, const double LEFT_OFFSET, const double HORIZONTAL_OFFSET,
            const double MOTOR_GEAR_TEETH, const double WHEEL_GEAR_TEETH, const double TRACKING_WHEEL_DIAMETER, pros::Motor& front_left,
            pros::Motor& middle_left, pros::Motor& back_left, pros::Motor& front_right, pros::Motor& middle_right, pros::Motor& back_right,
            pros::IMU& imu, pros::adi::Encoder& vertical, pros::adi::Encoder& horizontal, pros::Controller& controller, PID drive_pid,
            PID turn_pid
        );
        void set_drive_voltages(double left_voltage, double right_voltage);
        void set_drive_voltages(double voltage);
        void brake();
        void split_arcade();
        void drive_distance(double target, double max_voltage = 127, double max_acceleration = 6);
        void turn_to_heading(double degrees, double max_voltage = 127);
        void drive_to_point(double target_x, double target_y, double max_drive_voltage = 127, double max_turn_voltage = 127);
        void update_odometry();
        void set_original_heading(double new_original_heading);
        double get_heading();
};
#endif