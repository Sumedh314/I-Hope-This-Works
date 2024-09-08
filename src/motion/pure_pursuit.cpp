#include "main.h"
#include "motion/pure_pursuit.hpp"
#include "motion/odometry.hpp"
#include "motion/point.hpp"
#include "motion/math.hpp"
#include "subsystems/robot-config.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/drive.hpp"

void pure_pursuit(double path[25][2], int path_length, int forward_voltage, int direction) {
    double lookahead_radius = 9;

    int last_index = 1;
    
    double kP = 0.4;

    Point prev_point(0, 0);
    Point current_point(0, 0);

    Point intersect1(0, 0);
    Point intersect2(0, 0);

    Point lookahead_point(0, 0);
    printf("Start function.\n");
    // Run while the point the robot is following is on the path
    while (last_index < path_length) {
        // printf("\nStart loop.\n");
        // Set endpoints for the line segment the robot is on
        prev_point.set_x(path[last_index - 1][0]);
        prev_point.set_y(path[last_index - 1][1]);
        current_point.set_x(path[last_index][0]);
        current_point.set_y(path[last_index][1]);

        // If the line is vertical
        if (current_point.get_x() - prev_point.get_x() == 0) {
            current_point.set_x(current_point.get_x() + 0.003);
        }

        // Find line equation
        double slope = (current_point.get_y() - prev_point.get_y()) / (current_point.get_x() - prev_point.get_x());
        double y_int = current_point.get_y() - slope * current_point.get_x();

        // Set quadratic coefficients
        double quadratic_a = (1 + pow(slope, 2));
        double quadratic_b = (-2 * robot.get_x() - 2 * slope * robot.get_y() + 2 * slope * y_int);
        double quadratic_c = (pow(robot.get_x(), 2) + pow(y_int, 2) - 2 * y_int * robot.get_y() + pow(robot.get_y(), 2) - pow(lookahead_radius, 2));

        // Set minimum and maximum values for line segment endpoints
        double min_x = std::min(prev_point.get_x(), current_point.get_x());
        double max_x = std::max(prev_point.get_x(), current_point.get_x());

        // If discriminant is nonnegative
        if (pow(quadratic_b, 2) - 4 * quadratic_a * quadratic_c >= 0) {

            // Find intersection points
            intersect1.set_x((-quadratic_b + sqrt(pow(quadratic_b, 2) - 4 * quadratic_a * quadratic_c)) / (2 * quadratic_a));
            intersect1.set_y(intersect1.get_x() * slope + y_int);
            intersect2.set_x((-quadratic_b - sqrt(pow(quadratic_b, 2) - 4 * quadratic_a * quadratic_c)) / (2 * quadratic_a));
            intersect2.set_y(intersect2.get_x() * slope + y_int);

            // If at least one is in range
            if (intersect1.get_x() >= min_x && intersect1.get_x() <= max_x || intersect2.get_x() >= min_x && intersect2.get_x() <= max_x) {

                // If both are in range
                if (intersect1.get_x() >= min_x && intersect1.get_x() <= max_x && intersect2.get_x() >= min_x && intersect2.get_x() <= max_x) {
                    
                    // Find which intersection point is closer to the end of the path and set that to the lookahead point
                    if (distance_between_points(intersect1, current_point) < distance_between_points(intersect2, current_point)) {
                        lookahead_point.set_x(intersect1.get_x());
                        lookahead_point.set_y(intersect1.get_y());
                    }
                    else {
                        lookahead_point.set_x(intersect2.get_x());
                        lookahead_point.set_y(intersect2.get_y());
                    }

                    // printf("\nDistance to first point: \n%f", distance_between_points(intersect1, next_point));
                    // printf("\nDistance to second point: \n%f", distance_between_points(intersect2, next_point));
                }
                
                // Set the first intersection point as the lookahead point if only the first intersection point is in range
                else if (intersect1.get_x() >= min_x && intersect1.get_x() <= max_x) {
                    lookahead_point.set_x(intersect1.get_x());
                    lookahead_point.set_y(intersect1.get_y());
                }
                
                // Set the second intersection point as the lookahead point if only the first intersection point is in range
                else {
                    lookahead_point.set_x(intersect2.get_x());
                    lookahead_point.set_y(intersect2.get_y());
                }

                printf("Lookahead x: %f\n", lookahead_point.get_x());
                printf("Lookahead y: %f\n", lookahead_point.get_y());
            }
            
            // If neither intersection point is in range
            else if ((intersect1.get_x() < min_x || intersect1.get_x() > max_x) && (intersect2.get_x() < min_x || intersect2.get_x() > max_x)) {
                last_index++;
                printf("Neither is in range.\n");
                printf("Last index: %d\n", last_index);
                continue;
            }
        }

        // If robot doesn't intersect the line
        else {
            printf("\nDiscriminant is negative.\n");
        }
        

        // Go to the next line if the robot is nearing the end of the current line
        if (distance_between_points(robot, current_point) < distance_between_points(lookahead_point, current_point)) {
            last_index++;
            printf("Last index: %d\n", last_index);
            continue;
        }

        // printf("\nLookahead x: %f\n", lookahead_point.get_x());
        // printf("Lookahead y: %f\n", lookahead_point.get_y());

        // Get current and target headings
        double current_heading = robot.get_heading();
        double target_heading = rad_to_deg(atan2(lookahead_point.get_y() - robot.get_y(), lookahead_point.get_x() - robot.get_x()));

        // Calculate turn error
        if (direction == -1) {
            target_heading += 180;
        }
        double turn_error = target_heading - current_heading;

        // Reduce error -180 to 180
        while (turn_error > 180) {
            turn_error -= 360;
        }
        while (turn_error <= -180) {
            turn_error += 360;
        }
        printf("Turn error: %f\n", turn_error);

        // Move robot
        // double turn_voltage = TRACK_WIDTH * sin(deg_to_rad(turn_error)) / lookahead_radius * forward_voltage;
        double turn_voltage = turn_error * kP;
        double new_forward_voltage = forward_voltage - 2 * turn_voltage;
        set_drive_voltages(-turn_voltage + new_forward_voltage * direction, turn_voltage + new_forward_voltage * direction);

        pros::Task::delay(10);
    }

    // Stop robot
    set_drive_voltages(0, 0);
}