#include "main.h"
#include "motion/pid.hpp"
#include "subsystems/robot-config.hpp"
#include "subsystems/drive.hpp"

PID::PID(double new_kP, double new_kI, double new_kD, double new_start_i) {
    kP = new_kP;
    kI = new_kI;
    kD = new_kD;
    start_i = new_start_i;
}

double PID::compute(double error, double settle_error) {
    if (fabs(error) <= start_i) {
        total_error += error;
    }
    if (error >= 0 && prev_error <= 0 || error <= 0 && prev_error >= 0) {
        total_error = 0;
    }
    
    derivative = error - prev_error;

    power = error * kP + total_error * kI + derivative * kD;

    if (fabs(error) < settle_error) {
        time_spent_settling += 10;
    }
    else {
        time_spent_settling = 0;
    }

    timeout += 10;

    prev_error = error;
    return power;
}

bool PID::is_settled(double settle_time) {
    return time_spent_settling >= settle_time || timeout >= 4000;
}