#include "main.h"
#include "util.hpp"
#include "drive.hpp"
#include "pid.hpp"

/**
 * Creates a new Drive object with constant values.
*/
Drive::Drive(
    const double DRIVE_WHEEL_DIAMETER, const double TRACK_WIDTH, const double LEFT_OFFSET, const double HORIZONTAL_OFFSET,
    const double MOTOR_GEAR_TEETH, const double WHEEL_GEAR_TEETH, const double TRACKING_WHEEL_DIAMETER, pros::Motor& front_left,
    pros::Motor& middle_left, pros::Motor& back_left, pros::Motor& front_right, pros::Motor& middle_right, pros::Motor& back_right,
    pros::IMU& inertial, pros::adi::Encoder& vertical, pros::adi::Encoder& horizontal, PID drive_pid, PID turn_pid, pros::Controller& controller
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
    front_right(front_right),
    back_left(back_left),
    middle_right(middle_right),
    back_right(back_right),
    inertial(inertial),
    vertical(vertical),
    horizontal(horizontal),
    drive_pid(drive_pid),
    turn_pid(turn_pid),
    controller(controller)
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
        pros::delay(30);
    }
}

/**
 * Given a distance to drive, it calculates how much time it can accelerate, go at full speed, and then decelerate
 * to drive that distance. Doesn't work well, but creates a very smooth motion.
*/
void Drive::trapezoid_motion_profile(double displacement, double max_acceleration) {
    double distance = fabs(displacement);

    double voltage = 0;
    // double max_rpm = 732;
    double max_rpm = 640;
    double max_speed = max_rpm * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH * DRIVE_WHEEL_DIAMETER * pi / 60;
    double acceleration_time = 127 / max_acceleration / 100;
    double acceleration = max_speed / acceleration_time;
    double distance_accelerated = 0.5 * acceleration * pow(acceleration_time, 2);
    double wait_distance = distance - 2 * distance_accelerated;
    double wait_time = wait_distance / max_speed;

    if (wait_distance < 0) {
        acceleration_time -= fabs(wait_time) / 2;
        wait_time = 0;
    }

    printf("Max speed: %f\n", max_speed);
    printf("Acceleration time: %f\n", acceleration_time);
    printf("Acceleration: %f\n", acceleration);
    printf("Distance accelerated: %f\n", distance_accelerated);
    printf("Wait distance: %f\n", wait_distance);
    printf("Wait time: %f\n", wait_time);

    if (displacement < 0) {
        max_acceleration *= -1;
    }

    double start = pros::millis() / 1000.0;
	while (pros::millis() / 1000.0 < start + acceleration_time) {
		voltage += max_acceleration;
		set_drive_voltages(voltage);
        printf("%f\n", voltage);
		pros::delay(10);
	}

    start = pros::millis() / 1000.0;
    while (pros::millis() / 1000.0 < start + wait_time) {
        printf("%f\n", voltage);
        pros::delay(10);
    }
    
    start = pros::millis() / 1000.0;
	while (pros::millis() / 1000.0 < start + acceleration_time) {
		voltage -= max_acceleration;
		set_drive_voltages(voltage);
        printf("%f\n", voltage);
		pros::delay(10);
	}
    printf("Voltage: %f\n", voltage);
	brake();
}

/**
 * Uses the PID class to drive a certain distance. First accelerates to full speed smoothly and then begins the PID.
 * TODO: figure out correct PID constants.
*/
void Drive::drive_distance(double target, double max_voltage, double settle_error, double max_acceleration) {
    double error = target;
    double prev_error = target;
    double voltage = sign(error);
    double position = 0;

    // Set starting position of the motor to 0
    front_left.set_zero_position(0);

    // Accelerates at the beginning using the max acceleration. This is something our PID does not do, so we need to do
    // it separately.
    while (fabs(voltage) < max_voltage) {

        // Increase by max acceleration in the intended direction.
        voltage += max_acceleration * sign(voltage);

        // Set motor voltages.
        set_drive_voltages(voltage);
        
        // Update position and previous error to the current error.
        position = front_left.get_position() * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH * DRIVE_WHEEL_DIAMETER * pi / 360;
        prev_error = target - position;
        pros::delay(10);
    }

    // Set the error for the PID.
    drive_pid.set_error(error);

    // Keep going until the robot is settled, either by reaching the desired distance or by getting stuck for too long.
    while (!drive_pid.is_settled()) {

        // Get the current error and feet it into the PID controller.
        position = front_left.get_position() * MOTOR_GEAR_TEETH / WHEEL_GEAR_TEETH * DRIVE_WHEEL_DIAMETER * pi / 360;
        error = target - position;
        voltage = drive_pid.compute(error);

        // Clamp the voltage to the maximum voltage.
        if (fabs(voltage) > max_voltage) {
            voltage = max_voltage * sign(voltage);
            printf("voltage: %f\n", voltage);
        }

        // printf("volt: %f\n", voltage);
        // Output voltage and delay for next loop.
        set_drive_voltages(voltage);
        pros::delay(10);
    }
    
    // Make sure robot doesn't continue moving.
    brake();
}

/**
 * Uses the PID class to turn to a certain heading. First accelerates to full speed smoothly and then begins the PID.
 * TODO: figure out correct PID constants.
*/
void Drive::turn_to_heading(double target, double max_voltage, double max_acceleration) {
    double prev_error = target;
    double position = get_heading();
    double error = target - position;
    double voltage = sign(error);

    // double kP = 0;
    // if (fabs(error) <= 95) {
    //     kP = 1.6;
    // }
    // else {
    //     kP = 1;
    // }
    // double kD = 0;

    // Accelerates at the beginning using the max acceleration. This is something our PID does not do, so we need to do
    // it separately.
    while (fabs(voltage) < max_voltage) {

        // Increase by max acceleration in the intended direction
        voltage += max_acceleration * sign(voltage);

        // Set motor voltages.
        set_drive_voltages(-voltage, voltage);
        
        // Update position and previous error to the current error.
        position = get_heading();
        prev_error = target - position;
        pros::delay(10);
    }

    // Figure out current error and set it to the PID.
    error = target - position;
    turn_pid.set_error(error);

    // Keep going until the robot is settled, either by reaching the desired distance or by getting stuck for too long.
    while (!turn_pid.is_settled()) {
        // Get the current error and feet it into the PID controller.
        position = get_heading();
        error = target - position;
        voltage = turn_pid.compute(error);

        // Clamp the voltage to the maximum voltage.
        if (fabs(voltage) > max_voltage) {
            voltage = max_voltage * sign(voltage);
        }

        // Output voltage and delay for next loop.
        set_drive_voltages(-voltage, voltage);
        pros::delay(10);
    }

    // Make sure robot doesn't continue moving.
    brake();
}

/**
 * Uses PID and odometry to drive the robot to a point.
*/
void Drive::drive_to_point(double x, double y) {

    // Set error to be a big number so the PID is not settled and the while loop will start.
    drive_pid.set_error(100);

    // Make the target a Point object.
    Point target(x, y);

    // Keep going until the robot is settled, either by reaching the desired point or by getting stuck for too long.
    while (!drive_pid.is_settled()) {
        
        // Find errors in the distance and angle it needs to turn to to get to the desired point.
        double lateral_error = distance_between_points(get_position(), target);
        double turn_error = reduce_negative_180_to_180(rad_to_deg(atan2(y - get_y(), x - get_x()) - deg_to_rad(get_heading())));

        // Reverse turning and driving if it's faster for the robot to drive backwards.
        if (fabs(turn_error) > 90) {
            lateral_error *= -1;
            turn_error -= 180 * sign(turn_error);
        }

        // Use the PID class to get the voltages.
        double drive_voltage = drive_pid.compute(lateral_error);
        double turn_voltage = turn_pid.compute(turn_error);

        // Scale the drive voltage to be smaller based on how much the robot is facing the target.
        drive_voltage *= cos(deg_to_rad(turn_error));

        // Move the robot and delay for next loop.
        set_drive_voltages(drive_voltage - turn_voltage, drive_voltage + turn_voltage);
        pros::delay(10);
    }
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
        set_x(get_x() + local_y_offset * cos(average_heading) + local_x_offset * sin(average_heading));
        set_y(get_y() + local_y_offset * sin(average_heading) - local_x_offset * cos(average_heading));

        // Reset values for next loop.
        prev_vertical = current_vertical;
        prev_horizontal = current_horizontal;
        prev_heading = current_heading;

        // Using delay_until() to ensure exactly 10 milliseconds for each iteration instead of 10 milliseconds between
        // iterations. This is to keep consistency.
        pros::Task::delay_until(&start, 10);
    }
}

/**
 * Returns the heading of the robot.
*/
double Drive::get_heading() {
    // The reading from the sensor is scaled a little bit because these sensors usually do not return exactly 360 when
    // spun around 360 degrees. This scaling factor was determined experimentally by spinning the robot around a lot
    // and seeing what the reading was.
    return original_heading - inertial.get_rotation() * 3600 / 3595;
}

/**
 * Sets the heading of the robot when it starts the match.
*/
void Drive::set_original_heading(double new_original_heading) {
    original_heading = new_original_heading;
}

Point Drive::get_position() {
    Point position(get_x(), get_y());
    return position;
}