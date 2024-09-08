#ifndef PID_H
#define PID_H
#include "main.h"

class PID {
    double error = 100;
    double target = 0;
    double prev_error = 0;
    double total_error = 0;
    double derivative = 0;

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double start_i = 0;

    double power = 0;

    double heading = 0;

    double time_spent_settling = 0;
    double timeout = 0;

    public:
        PID(double new_kP, double new_kI, double new_kD, double new_start_i);
        double compute(double error, double settle_error);
        bool is_settled(double settle_time);
};

#endif