#ifndef DRIVE_HPP
#define DRIVE_HPP
#include "main.h"
#include "pid.hpp"
#include "point.hpp"

/**
 * A class representing a drivetrain. Includes odometry and motion algorithms such as pure pursuit and driving to a point.
*/
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
    pros::Motor& front_left;
    pros::Motor& middle_left;
    pros::Motor& front_right;
    pros::Motor& back_left;
    pros::Motor& middle_right;
    pros::Motor& back_right;

    pros::IMU& inertial;
    pros::Rotation& vertical;
    pros::Rotation& horizontal;

    pros::Controller& controller;

    // PID objects used for this drivetrain.
    PID drive_pid_IME;
    PID drive_pid;
    PID turn_pid;

    public:
        Drive(
            const double DRIVE_WHEEL_DIAMETER, const double TRACK_WIDTH, const double LEFT_OFFSET, const double HORIZONTAL_OFFSET,
            const double MOTOR_GEAR_TEETH, const double WHEEL_GEAR_TEETH, const double TRACKING_WHEEL_DIAMETER, pros::Motor& front_left,
            pros::Motor& middle_left, pros::Motor& back_left, pros::Motor& front_right, pros::Motor& middle_right, pros::Motor& back_right,
            pros::IMU& inertial, pros::Rotation& vertical, pros::Rotation& horizontal, pros::Controller& controller, PID drive_pid_IME,
            PID drive_pid, PID turn_pid
        );
        void set_drive_voltages(double left_voltage, double right_voltage);
        void set_drive_voltages(double voltage);
        void brake();
        void split_arcade();
        void drive_distance_with_IME(double target, double max_voltage = 127, double max_acceleration = 6);
        void drive_distance(double target, double max_voltage = 127);
        void drive_to_point(double target_x, double target_y, int direction = 0, double max_drive_voltage = 127, double max_turn_voltage = 127, double turn_limit = 5);
        void turn_to_heading(double degrees, int direction = 0, double max_voltage = 127);
        void turn_to_point(double target_x, double target_y, int direction = 0, double max_voltage = 127);
        void turn_and_drive_to_point(double target_x, double target_y, int turn_direction = 0, int drive_direction = 0, double max_drive_voltage = 127, double max_turn_voltage = 127);
        void follow_path(double path[25][2], int path_length, int forward_voltage = 127, int direction = 1);
        void update_odometry();
        void IMU_odometry();
        void reset_odometry();
        void set_original_heading(double original_heading);
        double get_heading();
};
#endif