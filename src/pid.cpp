#include "main.h"
#include "pid.hpp"

/**
 * Creates a new PID object. This is better than programming it manually every time since we can use a PID for much
 * more than just a drivetrain, so we don't have to program it manually every time.
*/
PID::PID(double kP, double kI, double kD, double start_i, double settle_error, double settling_speed, double timeout, double delay_time) :
    kP(kP),
    kI(kI),
    kD(kD),
    start_i(start_i),
    settle_error(settle_error),
    settling_speed(settling_speed),
    timeout(timeout),
    delay_time(delay_time),
    settled_flag(false)
{
    error = settle_error + 100;
}

/**
 * Uses the error, total error over time, and the current speed to compute an output value to feed into the mechanism.
*/ 
double PID::compute(double error) {
    this->error = error;

    // Only start counting the total error if the robot is already close to the target.
    if (fabs(error) <= start_i) {
        total_error += error * delay_time;
    }
    // Set integral back to zero if the robot crosses the target to prevent it from accumulating too much.
    if (error >= 0 && prev_error <= 0 || error <= 0 && prev_error >= 0) {
        total_error = 0;
    }
    
    // Current speed of the mechanism.
    derivative = (error - prev_error) / delay_time;

    // Reset previous error for next loop.
    prev_error = error;

    // If the robot stops moving (e.g. if it gets stuck) start counting how long it is stuck.
    if (fabs(derivative) < settling_speed) {
        time_settling += delay_time * 1000;
    }
    // If the robot is moving, reset time_settling.
    else {
        time_settling = 0;
    }

    // Calculate and return output.
    output = kP * error + kI * total_error + kD * derivative;
    return output;
}

/**
 * The robot is settled if its error is within the desired range and its speed is zero, or if it gets stuck somewhere
 * for too long. This is so that even if the robot gets stuck, it can move on to the rest of the autonomous routine
 * and hopefully still score points.
*/
bool PID::is_settled() {
    if (fabs(error) <= settle_error && fabs(derivative) <= settling_speed || time_settling >= timeout) {
        error = settle_error + 100;
        return true;
    }
    return false;
}