#include "main.h"
#include "util.hpp"
#include "robot_subsystems/drive.hpp"
#include "pid.hpp"

/**
 * Creates a new Drive object with constant values representing its dimensions, values related to tracking wheels,
 * information about electronics it uses, and PID objects. Sets the initial position to (0, 0).
*/
Drive::Drive(
    const double DRIVE_WHEEL_DIAMETER, const double TRACK_WIDTH, const double LEFT_OFFSET, const double HORIZONTAL_OFFSET,
    const double MOTOR_GEAR_TEETH, const double WHEEL_GEAR_TEETH, const double TRACKING_WHEEL_DIAMETER, pros::Motor& front_left,
    pros::Motor& middle_left, pros::Motor& back_left, pros::Motor& front_right, pros::Motor& middle_right, pros::Motor& back_right,
    pros::IMU& inertial, pros::adi::Encoder& vertical, pros::adi::Encoder& horizontal, pros::Controller& controller,
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
    inertial(inertial),
    vertical(vertical),
    horizontal(horizontal),
    controller(controller),
    drive_pid_IME(drive_pid_IME),
    drive_pid(drive_pid),
    turn_pid(turn_pid),
    Point(0, 0)
{}

/**
 * Sets the voltages of the drivetrain motors. the PROS motor.move() function takes in a value from -127 to 127 and
 * scales it to be between -12 and 12 because those are the actual voltage limit for V5 motors.
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
 * A function overload for the function above. Useful for setting both sides of the drivetrain to the same value.
*/
void Drive::set_drive_voltages(double voltage) {
    set_drive_voltages(voltage, voltage);
}

/**
 * Stops the drivetrain by setting the voltage values to 0.
*/
void Drive::brake() {
    set_drive_voltages(0);
}

/**
 * Controls the robot with the controller using split arcade control. In this control, the left joystick makes the
 * robot move forward and backwards, and the right joystick makes it turn left and right. The values from the
 * joysticks are also scaled using a quadratic curve to make smaller inputs more precise but still get full throttle
 * when the joysticks are pushed to their physical limits.
*/
void Drive::split_arcade() {
    while (true) {
        double left_value = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double right_value = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        double left_voltage = left_value * fabs(left_value) / 127 + right_value * fabs(right_value) / 127;
        double right_voltage = left_value * fabs(left_value) / 127 - right_value * fabs(right_value) / 127;

        set_drive_voltages(left_voltage, right_voltage);
        pros::delay(20);
    }
}

/**
 * Uses the PID class to drive a certain distance. First accelerates to full speed smoothly and then begins the PID.
 * This function uses the Integrated Motor Encoders to get the position of the wheels and does not use odometry.
 * TODO: figure out correct PID constants.
*/
void Drive::drive_distance_with_IME(double target, double max_voltage, double max_acceleration) {
    double error = target;
    double prev_error = target;
    double voltage = sign(error);
    double position = 0;

    // Set starting position of the motor to 0 degrees.
    front_left.set_zero_position(0);

    // Accelerates at the beginning using the max acceleration. This is something our PID does not do, so we need to do
    // it separately since the wheels could slip.
    while (fabs(voltage) < max_voltage) {

        // Increase voltage by max acceleration in the intended direction and move the robot.
        voltage += max_acceleration * sign(voltage);
        set_drive_voltages(voltage);
        
        // Update position and previous error to the current error.
        position = front_left.get_position() * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH * DRIVE_WHEEL_DIAMETER * pi / 360;
        prev_error = target - position;
        pros::delay(10);
    }

    // Keep going until the robot is settled, either by reaching the desired distance or by getting stuck for too long.
    while (!drive_pid_IME.is_settled()) {

        // Get the current error and feet it into the PID controller. Position and error are in inches.
        position = front_left.get_position() * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH * DRIVE_WHEEL_DIAMETER * pi / 360;
        error = target - position;
        voltage = drive_pid_IME.compute(error);

        // Clamp the voltage to the maximum voltage.
        voltage = clamp(voltage, max_voltage);

        // printf("volt: %f\n", voltage);
        // Output voltage and delay for next loop.
        set_drive_voltages(voltage);
        pros::delay(10);
    }
    
    // Make sure robot doesn't continue moving.
    brake();
}

/**
 * Uses the PID class to drive a certain distance. This function uses the vertical tracking wheel to get the distance. This
 * should eliminate the need to accelerate smoothly at the beginning of the motion because the tracking wheel shouldn't slip.
 * TODO: figure out correct PID constants.
*/
void Drive::drive_distance(double target, double max_voltage) {

    // Get the original position of the encoder in degrees.
    double original_position = vertical.get_value();

    // Keep going until the robot if settled, either by reaching the desired distance or by getting stuck for too long.
    while (!drive_pid.is_settled()) {

        // Get the current error in inches and feed it into the PID controller. Looks at the difference in the current
        // position and the original position and converts that to inches.
        double current_position = (vertical.get_value() - original_position) * TRACKING_WHEEL_DIAMETER * pi / 360;
        double error = target - current_position;
        double voltage = drive_pid.compute(error);

        // Clamp the voltage to the allowed range.
        voltage = clamp(voltage, max_voltage);
        // double voltage = 0;

        // Output voltages and delay for next loop.
        set_drive_voltages(voltage);
        pros::delay(10);
    }

    // Make sure robot doesn't continue moving.
    brake();
}

/**
 * Uses PID and odometry to drive the robot to a point on the field.
 * TODO: figure out correct PID constants.
*/
void Drive::drive_to_point(double target_x, double target_y, bool reversed, double max_drive_voltage, double max_turn_voltage, double turn_limit) {
    double turn_voltage = 0;

    // Make the target a Point object.
    Point target(target_x, target_y);

    // Keep going until the robot is settled, either by reaching the desired point or by getting stuck for too long.
    while (!drive_pid.is_settled()) {
        
        // Find errors in the distance and angle it needs to turn to to get to the desired point.
        double lateral_error = distance_between_points(*this, target);
        double turn_error = reduce_negative_180_to_180(
            rad_to_deg(atan2(target.get_y() - y, target.get_x() - x) - deg_to_rad(get_heading()))
        );

        // Reverse turning and driving so the robot drives backwards if the back of the robot is facing the target.
        if (fabs(turn_error) > 90 || reversed) {
            lateral_error *= -1;
            turn_error -= 180 * sign(turn_error);
        }

        // Use the PID class to get the voltages.
        double drive_voltage = drive_pid.compute(lateral_error);

        // Only calculate turn voltage if the lateral error is large. This is to help prevent the robot turning a lot
        // near the end.
        if (fabs(lateral_error) > turn_limit) {
            turn_voltage = turn_pid.compute(turn_error);
        }
        else {
            turn_voltage = 0;
        }

        // Scale the drive voltage to be smaller based on how much the robot is facing the target. This way, the robot
        // will drive forward slower if it is not directly facing the point.
        drive_voltage *= cos(deg_to_rad(turn_error / 2));

        // Keep the voltages within the limits given by the parameters.
        drive_voltage = clamp(drive_voltage, max_drive_voltage);
        turn_voltage = clamp(turn_voltage, max_turn_voltage);

        // Prevent the sum of drive_voltage and turn_voltage from being greater than the maximum allowed voltage.
        if (fabs(drive_voltage) + fabs(turn_voltage) > 127) {

            // Essentially the same as drive_voltage = 127 - turn_voltage, but accounts for sign changes.
            drive_voltage = (127 - fabs(turn_voltage)) * sign(drive_voltage);
        }

        // Move the robot and delay for next loop.
        set_drive_voltages(drive_voltage - turn_voltage, drive_voltage + turn_voltage);
        pros::delay(10);
    }
    turn_pid.compute(100);
}

/**
 * Uses the PID class to turn to a certain heading. First accelerates to full speed smoothly and then begins the PID.
 * TODO: figure out correct PID constants.
*/
void Drive::turn_to_heading(double target, double max_voltage) {

    // Keep going until the robot is settled, either by reaching the desired distance or by getting stuck for too long.
    while (!turn_pid.is_settled()) {

        // Get the current error and feet it into the PID controller.
        double position = get_heading();
        double error = reduce_negative_180_to_180(target - position);
        double voltage = turn_pid.compute(error);

        // Clamp the voltage to the allowed range.
        voltage = clamp(voltage, max_voltage);

        // Output voltages and delay for next loop.
        set_drive_voltages(-voltage, voltage);
        pros::delay(10);
    }

    // Make sure robot doesn't continue moving.
    brake();
}

void Drive::turn_to_point(double target_x, double target_y, double max_voltage) {

    // Make the target a Point object.
    Point target(target_x, target_y);

    // Keep going until the robot is settled, either by reaching the desired distance or by getting stuck for too long.
    while (!turn_pid.is_settled()) {

        // Get the current error and feet it into the PID controller.
        double position = get_heading();
        double error = rad_to_deg(atan2(target.get_y() - y, target.get_x() - x) - deg_to_rad(get_heading()));
        double voltage = turn_pid.compute(error);

        // Clamp the voltage to the allowed range.
        voltage = clamp(voltage, max_voltage);

        // Output voltages and delay for next loop.
        set_drive_voltages(-voltage, voltage);
        pros::delay(10);
    }

    // Make sure robot doesn't continue moving.
    brake();
}

/**
 * Uses the pure pursuit algorithm to follow a path.
 * NOTE: this probably does not work well at all. It is almost completely copied and pasted from last year's code. I
 * mostly used this guide to make this function: https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit
*/
void Drive::follow_path(double path[25][2], int path_length, int forward_voltage, int direction) {

    // The radius around which the robot finds intersection points with the path. I just chose a value for this.
    double lookahead_radius = 9;

    // The index for the points on the path. Starts at 1 and iterates through the path.
    int last_index = 1;
    
    // A proportional constant for adjusting the heading. I just chose a value for this that worked.
    double kP = 0.4;

    // The endopints of the line from the path that the robot is currently following.
    Point prev_point(0, 0);
    Point current_point(0, 0);

    // The two points at which the robot's circle intersects the path.
    Point intersect_1(0, 0);
    Point intersect_2(0, 0);

    // The point that the robot will try to follow.
    Point lookahead_point(0, 0);
    printf("Start function.\n");

    // Run the loop while the point the robot is following is on the path.
    while (last_index < path_length) {
        // printf("\nStart loop.\n");

        // Set endpoints for the line segment the robot is on.
        prev_point.set_x(path[last_index - 1][0]);
        prev_point.set_y(path[last_index - 1][1]);
        current_point.set_x(path[last_index][0]);
        current_point.set_y(path[last_index][1]);

        // If the line is vertical, make it slightly not vertical.
        if (current_point.get_x() - prev_point.get_x() == 0) {
            current_point.set_x(current_point.get_x() + 0.01);
        }

        // Find the equation of the line the robot is following in slope-intercept form.
        double slope = (current_point.get_y() - prev_point.get_y()) / (current_point.get_x() - prev_point.get_x());
        double y_int = current_point.get_y() - slope * current_point.get_x();

        // Set quadratic coefficients. This was derived by setting the equations for a circle and a line equal to each other.
        double quadratic_a = (1 + pow(slope, 2));
        double quadratic_b = (-2 * x - 2 * slope * y + 2 * slope * y_int);
        double quadratic_c = (pow(x, 2) + pow(y_int, 2) - 2 * y_int * y + pow(y, 2) - pow(lookahead_radius, 2));

        // Set minimum and maximum values for line segment endpoints.
        double min_x = std::min(prev_point.get_x(), current_point.get_x());
        double max_x = std::max(prev_point.get_x(), current_point.get_x());

        // If discriminant is nonnegative (e.g. the robot's circle intersects the line), start the algorithm.
        if (pow(quadratic_b, 2) - 4 * quadratic_a * quadratic_c >= 0) {

            // Find the intersection points using the quadratic formula.
            intersect_1.set_x((-quadratic_b + sqrt(pow(quadratic_b, 2) - 4 * quadratic_a * quadratic_c)) / (2 * quadratic_a));
            intersect_1.set_y(intersect_1.get_x() * slope + y_int);
            intersect_2.set_x((-quadratic_b - sqrt(pow(quadratic_b, 2) - 4 * quadratic_a * quadratic_c)) / (2 * quadratic_a));
            intersect_2.set_y(intersect_2.get_x() * slope + y_int);

            // If at least one intersection point is in range, figure out which one to follow.
            if (intersect_1.get_x() >= min_x && intersect_1.get_x() <= max_x || intersect_2.get_x() >= min_x && intersect_2.get_x() <= max_x) {

                // If both are in range, choose the one farther along the path.
                if (intersect_1.get_x() >= min_x && intersect_1.get_x() <= max_x && intersect_2.get_x() >= min_x && intersect_2.get_x() <= max_x) {
                    
                    // Find which intersection point is closer to the end of the path and set that to the lookahead point.
                    if (distance_between_points(intersect_1, current_point) < distance_between_points(intersect_2, current_point)) {
                        lookahead_point.set_x(intersect_1.get_x());
                        lookahead_point.set_y(intersect_1.get_y());
                    }
                    else {
                        lookahead_point.set_x(intersect_2.get_x());
                        lookahead_point.set_y(intersect_2.get_y());
                    }

                    // printf("\nDistance to first point: \n%f", distance_between_points(intersect1, next_point));
                    // printf("\nDistance to second point: \n%f", distance_between_points(intersect2, next_point));
                }
                
                // Set the first intersection point as the lookahead point if only the first intersection point is in range.
                else if (intersect_1.get_x() >= min_x && intersect_1.get_x() <= max_x) {
                    lookahead_point.set_x(intersect_1.get_x());
                    lookahead_point.set_y(intersect_1.get_y());
                }
                
                // Set the second intersection point as the lookahead point if only the second intersection point is in range.
                else {
                    lookahead_point.set_x(intersect_2.get_x());
                    lookahead_point.set_y(intersect_2.get_y());
                }

                printf("Lookahead x: %f\n", lookahead_point.get_x());
                printf("Lookahead y: %f\n", lookahead_point.get_y());
            }
            
            // If neither intersection point is in range, move onto the next point.
            else if ((intersect_1.get_x() < min_x || intersect_1.get_x() > max_x) && (intersect_2.get_x() < min_x || intersect_2.get_x() > max_x)) {
                last_index++;
                printf("Neither is in range.\n");
                printf("Last index: %d\n", last_index);
                continue;
            }
        }

        // If robot doesn't intersect the line, do nothing.
        else {
            printf("\nDiscriminant is negative.\n");
        }
        

        // Go to the next line if the robot is nearing the end of the current line.
        if (distance_between_points(*this, current_point) < distance_between_points(lookahead_point, current_point)) {
            last_index++;
            printf("Last index: %d\n", last_index);
            continue;
        }

        // printf("\nLookahead x: %f\n", lookahead_point.get_x());
        // printf("Lookahead y: %f\n", lookahead_point.get_y());

        // Get current and target headings.
        double current_heading = get_heading();
        double target_heading = rad_to_deg(atan2(lookahead_point.get_y() - y, lookahead_point.get_x() - x));

        // Calculate turn error and reduce it to be betwen -180 and 180 degrees.
        if (direction == -1) {
            target_heading += 180;
        }
        double turn_error = reduce_negative_180_to_180(target_heading - current_heading);

        printf("Turn error: %f\n", turn_error);

        // Move robot
        // double turn_voltage = TRACK_WIDTH * sin(deg_to_rad(turn_error)) / lookahead_radius * forward_voltage;
        double turn_voltage = turn_error * kP;
        double new_forward_voltage = forward_voltage - 3 * fabs(turn_voltage);
        printf("forward voltage: %f\n", new_forward_voltage);
        printf("turn voltage: %f\n", turn_voltage);
        set_drive_voltages(-turn_voltage + new_forward_voltage * direction, turn_voltage + new_forward_voltage * direction);

        pros::Task::delay(10);
    }

    // Stop robot
    brake();
}

/**
 * Updates the position of the robot using tracking wheels. Uses an arc as an approximation of the robot's movement
 * over the past 10 milliseconds.
*/
void Drive::update_odometry() {
    double prev_heading = deg_to_rad(get_heading());
    double prev_vertical = 0;
    double prev_horizontal = 0;
    double local_x_offset = 0;
    double local_y_offset = 0;

    while (true) {

        // The time at which this iteration of the loop started.
        std::uint32_t start = pros::millis();

        // Find out how the robot changed from the previous loop.
        double current_vertical = vertical.get_value();
        double vertical_distance = (current_vertical - prev_vertical) * TRACKING_WHEEL_DIAMETER * pi / 360;
        double current_horizontal = horizontal.get_value();
        double horizontal_distance = (current_horizontal - prev_horizontal) * TRACKING_WHEEL_DIAMETER * pi / 360;
        double current_heading = deg_to_rad(get_heading());
        double change_in_heading = current_heading - prev_heading;

        // If the robot hasn't turned, the local offset is simply the distances traveled by the tracking wheels.
        if (change_in_heading == 0) {
            local_x_offset = horizontal_distance;
            local_y_offset = vertical_distance;
        }

        // If the robot has turned, the local offset is calculated using an arc approximation of the robot's movement.
        else {
            local_x_offset = 2 * sin(change_in_heading / 2) * (horizontal_distance / change_in_heading + HORIZONTAL_OFFSET);
            local_y_offset = 2 * sin(change_in_heading / 2) * (vertical_distance / change_in_heading + LEFT_OFFSET);
        }

        // Get the average heading of the robot in the previous 10 milliseconds. This is more accurate than using the
        // current heading.
        double average_heading = current_heading - change_in_heading / 2;

        // Change x and y coordinates using the local x and y offsets.
        x += local_y_offset * cos(average_heading) + local_x_offset * sin(average_heading);
        y += local_y_offset * sin(average_heading) - local_x_offset * cos(average_heading);

        // Reset values for next loop.
        prev_vertical = current_vertical;
        prev_horizontal = current_horizontal;
        prev_heading = current_heading;

        // Using delay_until() to ensure exactly 10 milliseconds for each iteration instead of 10 milliseconds between
        // iterations. For example, if this loop took 2 milliseconds to run, it would only wait 8 milliseconds before
        // the next loop instead of 10. This is to make it more consistent.
        pros::Task::delay_until(&start, 10);
    }
}

/**
 * Sets the heading of the robot when it starts the match. Should only be used at the beginning of the match becuase
 * the inertial sensor's reading will also be reset (but not recalibrated).
*/
void Drive::set_original_heading(double original_heading) {
    inertial.set_rotation(0);
    this->original_heading = original_heading;
}

/**
 * Returns the heading of the robot. The reading from the sensor is scaled a little bit because these sensors usually
 * do not return exactly 360 when spun around 360 degrees. This scaling factor was determined experimentally by
 * spinning the robot around a lot and seeing what the reading was.
*/
double Drive::get_heading() {
    return original_heading - inertial.get_rotation() * 3600 / 3595;
}