#include "main.h"
#include "util.hpp"
#include "robot_subsystems/drive.hpp"
#include "robot_subsystems/robot-config.hpp"
#include "pid.hpp"

/**
 * Creates a new Drive object with constants representing drivetrain geometry,
 * gear ratios, and PID controllers. No sensors are initialized here.
 */
Drive::Drive(
    const double DRIVE_WHEEL_DIAMETER, const double TRACK_WIDTH,
    const double LEFT_OFFSET, const double HORIZONTAL_OFFSET,
    const double MOTOR_GEAR_TEETH, const double WHEEL_GEAR_TEETH,
    const double TRACKING_WHEEL_DIAMETER,
    pros::Motor& front_left, pros::Motor& middle_left, pros::Motor& back_left,
    pros::Motor& front_right, pros::Motor& middle_right, pros::Motor& back_right,
    pros::Controller& controller,
    PID drive_pid_IME, PID drive_pid, PID turn_pid
) :
    DRIVE_WHEEL_DIAMETER(DRIVE_WHEEL_DIAMETER),
    TRACK_WIDTH(TRACK_WIDTH),
    LEFT_OFFSET(LEFT_OFFSET),
    HORIZONTAL_OFFSET(HORIZONTAL_OFFSET),
    MOTOR_GEAR_TEETH(MOTOR_GEAR_TEETH),
    WHEEL_GEAR_TEETH(WHEEL_GEAR_TEETH),
    TRACKING_WHEEL_DIAMETER(TRACKING_WHEEL_DIAMETER),
    front_left(front_left),
    middle_left(middle_left),
    back_left(back_left),
    front_right(front_right),
    middle_right(middle_right),
    back_right(back_right),
    controller(controller),
    drive_pid_IME(drive_pid_IME),
    drive_pid(drive_pid),
    turn_pid(turn_pid),
    Point(0, 0),
    original_heading(0.0)  // ADD INITIALIZATION
{}

/**
 * Sets motor voltages for each side of the drivetrain.
 */
void Drive::set_drive_voltages(double left_voltage, double right_voltage) {
    front_left.move(left_voltage);
    middle_left.move(left_voltage);
    back_left.move(left_voltage);

    front_right.move(right_voltage);
    middle_right.move(right_voltage);
    back_right.move(right_voltage);
}

/**
 * Sets both sides to the same voltage.
 */
void Drive::set_drive_voltages(double voltage) {
    set_drive_voltages(voltage, voltage);
}

/**
 * Stops the drivetrain.
 */
void Drive::brake() {
    set_drive_voltages(0);
}

/**
 * Split arcade drive control (left stick = forward/back, right stick = turn).
 */
void Drive::split_arcade() {
    double prev_left_value = 0;
    double slew = 1000;

    while (true) {
        double left_value = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double right_value = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (fabs(left_value) < 10) left_value = 0;
        if (fabs(right_value) < 10) right_value = 0;

        if (fabs(left_value - prev_left_value) > slew)
            left_value = prev_left_value + slew * sign(left_value - prev_left_value);

        double left_voltage = left_value * fabs(left_value) / 127 + right_value * fabs(right_value) / 127;
        double right_voltage = left_value * fabs(left_value) / 127 - right_value * fabs(right_value) / 127;

        set_drive_voltages(left_voltage, right_voltage);

        prev_left_value = left_value;
        pros::delay(20);
    }
}

/**
 * Uses motor IMEs and PID to drive a set distance (no tracking wheels or sensors).
 */
void Drive::drive_distance_with_IME(double target, double max_voltage, double max_acceleration) {
    double error = target;
    double prev_error = target;
    double voltage = sign(error);
    double position = 0;

    front_left.set_zero_position(0);

    // Accelerate smoothly
    while (fabs(voltage) < max_voltage) {
        voltage += max_acceleration * sign(voltage);
        set_drive_voltages(voltage);

        position = front_left.get_position() * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH *
                   DRIVE_WHEEL_DIAMETER * pi / 360;
        prev_error = target - position;
        pros::delay(10);
    }

    // PID control loop
    while (!drive_pid_IME.is_settled()) {
        position = front_left.get_position() * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH *
                   DRIVE_WHEEL_DIAMETER * pi / 360;
        error = target - position;
        voltage = drive_pid_IME.compute(error);
        voltage = clamp(voltage, max_voltage);
        set_drive_voltages(voltage);
        pros::delay(10);
    }

    brake();
}

/**
 * Uses PID to drive a set distance using the drive_pid (alternative to IME version)
 */
void Drive::drive_distance(double target, double max_voltage) {
    double error = target;
    double position = 0;

    // Reset encoder position
    front_left.set_zero_position(0);

    // PID control loop
    while (!drive_pid.is_settled()) {
        position = front_left.get_position() * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH *
                   DRIVE_WHEEL_DIAMETER * pi / 360;
        error = target - position;
        double voltage = drive_pid.compute(error);
        voltage = clamp(voltage, max_voltage);
        set_drive_voltages(voltage);
        pros::delay(10);
    }

    brake();
}

/**
 * Sets the original heading of the robot
 */
void Drive::set_original_heading(double heading) {
    original_heading = heading;
}

/**
 * Gets the current heading of the robot
 */
double Drive::get_heading() {
    // For now, return original heading since no IMU is available
    // You can replace this with actual heading calculation when IMU is added
    return original_heading;
}

/**
 * Uses PID to turn the robot to a given heading (assumes external heading logic if used).
 */
void Drive::turn_to_heading(double target, int direction, double max_voltage) {
    while (!turn_pid.is_settled()) {
        double position = get_heading(); // Use the heading method
        double error = reduce_negative_180_to_180(target - position);

        if ((fabs(error) > 90 || direction == -1) && direction != 1)
            error -= 180 * sign(error);

        double voltage = turn_pid.compute(error);
        voltage = clamp(voltage, max_voltage);

        set_drive_voltages(-voltage, voltage);
        pros::delay(10);
    }

    brake();
}

/**
 * Basic drive-to-point function stub (kept for compatibility, no odometry logic).
 */
void Drive::drive_to_point(double target_x, double target_y, int direction,
                           double max_drive_voltage, double max_turn_voltage, double turn_limit) {
    // Without odometry, this function cannot compute positions
    // You can integrate your own external position tracking system here if desired.
    // For now, just drive forward a calculated distance
    double distance = sqrt(pow(target_x - x, 2) + pow(target_y - y, 2));
    drive_distance(distance, max_drive_voltage);
}

/**
 * Turn toward a point (stub).
 */
void Drive::turn_to_point(double target_x, double target_y, int direction, double max_voltage) {
    // Calculate angle to target point (stub implementation)
    double angle = atan2(target_y - y, target_x - x) * 180 / M_PI;
    turn_to_heading(angle, direction, max_voltage);
}

/**
 * Combined turn and drive sequence.
 */
void Drive::turn_and_drive_to_point(double target_x, double target_y, int turn_direction,
                                    int drive_direction, double max_drive_voltage, double max_turn_voltage) {
    turn_to_point(target_x, target_y, turn_direction, max_turn_voltage);
    drive_to_point(target_x, target_y, drive_direction, max_drive_voltage, max_turn_voltage);
}

// Stub implementations for other methods
void Drive::follow_path(double path[25][2], int path_length, int forward_voltage, int direction) {
    // Stub - implement path following when odometry is available
}

void Drive::update_odometry() {
    // Stub - implement odometry update when sensors are available
}

void Drive::IMU_odometry() {
    // Stub - implement IMU-based odometry when IMU is available
}

void Drive::reset_odometry() {
    // Reset position to (0,0) and heading to original
    x = 0;
    y = 0;
    set_original_heading(0);
}