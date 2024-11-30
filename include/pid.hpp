#ifndef PID_HPP
#define PID_HPP
#include "main.h"

/**
 * Uses PID, which stands for Proportional, Integral, Derivative, to move something to a desired location. For example,
 * we can use PID to move the drivetrain a certain distance.
*/
class PID {

    // Variables to track the state of the movement
    double error;
    double total_error;
    double prev_error;

    // The error at which the robot can exit the PID loop.
    double settle_error;

    // PID constants
    double kP;
    double kI;
    double kD;

    // Error at which the total_error term starts accumulating. This is to prevent it from getting too large.
    double start_i;

    // The speed of the robot. Used in calculating the output power and for determining if the robot has settled or not.
    double derivative;

    // The amount of time the robot spends not moving and the maximum speed for it to be considered settled (should be
    // extremely small).
    double time_settling;
    double settling_speed;

    // The limit to how much time the robot can spend not moving before exiting out of the loop.
    double timeout;

    // The output commanded to the mechanism that's moving.
    double output;

    // The amount of time in milliseconds between each iteration of the loop.
    double delay_time;

    public:
        PID(double kP, double kI, double kD, double start_i, double settle_error, double settling_speed, double timeout, double delay_time);
        double compute(double error);
        bool is_settled();
};
#endif