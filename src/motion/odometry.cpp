#include "main.h"
#include "motion/odometry.hpp"
#include "motion/point.hpp"
#include "motion/math.hpp"
#include "subsystems/robot-config.hpp"

Point robot(0, 0, 90);

void odometry() {
    double original_heading = robot.get_heading();
    double prev_heading = deg_to_rad(robot.get_heading());
    double prev_left = 0;
    double local_y_offset = 0;

    while (true) {
        std::uint32_t start = pros::millis();

        double current_left = front_left.get_position();
        double left_distance = (current_left - prev_left) * WHEEL_DIAMETER * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH * pi / 360;

        double current_heading = deg_to_rad(robot.get_heading());
        double change_in_heading = current_heading - prev_heading;

        prev_left = current_left;
        prev_heading = current_heading;

        if (change_in_heading == 0) {
            local_y_offset = left_distance;
        }
        else {
            local_y_offset = 2 * sin(change_in_heading / 2) * (left_distance / change_in_heading + LEFT_OFFSET);
        }

        double average_heading = current_heading - change_in_heading / 2;

        robot.set_x(robot.get_x() + local_y_offset * cos(average_heading));
        robot.set_y(robot.get_y() + local_y_offset * sin(average_heading));
        robot.set_heading(original_heading - inertial.get_rotation() * 3600 / 3595);

        pros::Task::delay_until(&start, 10);
    }
}