#include "main.h"
#include "subsystems/robot-config.hpp"
#include "motion/pid.hpp"
#include "motion/odometry.hpp"
#include "motion/point.hpp"
#include "motion/math.hpp"

void print_information() {
	while (true) {
		controller.print(0, 0, "X: %f", robot.get_x());
		pros::Task::delay(50);
		controller.print(1, 0, "Y: %f", robot.get_y());
		pros::Task::delay(50);
		controller.print(2, 0, "Rotation: %f", robot.get_heading());
		pros::Task::delay(50);
		// controller.print(0, 0, "%f", flywheel.get_actual_velocity());
		// pros::Task::delay(50);
	}
}

void set_drive_voltages(double left_voltage, double right_voltage) {
    front_left.move(left_voltage);
    middle_left.move(left_voltage);
    back_left.move(left_voltage);
    
    front_right.move(right_voltage);
    middle_right.move(right_voltage);
    back_right.move(right_voltage);
}

void split_arcade() {
    while (true) {
        double left_value = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double right_value = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        double left_voltage = left_value / 127 * abs(left_value) + right_value / 127 * abs(right_value);
        double right_voltage = left_value / 127 * abs(left_value) - right_value / 127 * abs(right_value);

        set_drive_voltages(left_voltage, right_voltage);
        pros::Task::delay(30);
    }
}

void drive_for(double distance) {
    double max_acceleration = 6;
    double prev_power = 0;

    PID drive_PID(0.5, 0.04, 4.5, 5);
    double target = distance * 360 / (WHEEL_DIAMETER * pi) * WHEEL_GEAR_TEETH / MOTOR_GEAR_TEETH;
    double original_position = front_left.get_position();

    while (!drive_PID.is_settled(100)) {
        std::uint32_t start = pros::millis();
        double current_position = front_left.get_position();
        double power = drive_PID.compute(target - (current_position - original_position), 50);

        if (power > 0) {
            if (power - prev_power > max_acceleration) {
                power = prev_power + max_acceleration;
            }
        }
        else {
            if (prev_power - power > max_acceleration) {
                power = prev_power - max_acceleration;
            }
        }
        
        set_drive_voltages(power, power);
        prev_power = power;
        pros::Task::delay_until(&start, 10);
    }
    set_drive_voltages(0, 0);
}

void drive_to_point(double x, double y) {
    double turn_power = 0;

    PID drive_point_PID(7, 0.15, 50.0, 5);
    PID drive_point_turn_PID(0.3, 0.0, 15.0, 15);
    Point target(x, y);

    while (!drive_point_PID.is_settled(100)) {
        std::uint32_t start = pros::millis();

        double current_heading = robot.get_heading();

        double drive_error = distance_between_points(robot, target);
        double turn_error = rad_to_deg(atan2(target.get_y() - robot.get_y(), target.get_x() - robot.get_x())) - current_heading;

        if (turn_error > 180) {
            turn_error -= 360;
        }
        else if (turn_error <= -180) {
            turn_error += 360;
        }

        if (turn_error > 90) {
            turn_error -= 180;
            drive_error *= -1;
        }
        else if (turn_error < -90) {
            turn_error += 180;
            drive_error *= -1;
        }

        double drive_power = drive_point_PID.compute(drive_error, 3);
        if (drive_error > 5) {
            turn_power = drive_point_turn_PID.compute(turn_error, 1);
        }
        else {
            turn_power = 0;
        }

        if (drive_power + fabs(turn_power) > 127) {
            drive_power = 127 - fabs(turn_power);
        }
        if (drive_power - fabs(turn_power) < -127) {
            drive_power = -127 + fabs(turn_power);
        }

        set_drive_voltages(drive_power - turn_power, drive_power + turn_power);
        pros::Task::delay_until(&start, 10);
    }
    set_drive_voltages(0, 0);
}

void turn_to_heading(double target_heading) {
    PID turn_PID(2.2, 0.05, 13.0, 15);

    while (!turn_PID.is_settled(100)) {
        std::uint32_t start = pros::millis();

        double current_heading = robot.get_heading();
        double error = target_heading - current_heading;

        if (error > 180) {
            error -= 360;
        }
        else if (error <= -180) {
            error += 360;
        }

        double power = turn_PID.compute(error, 3);
        
        set_drive_voltages(-power, power);
        pros::Task::delay_until(&start, 10);
    }
    set_drive_voltages(0, 0);
}

void swing_left_to_heading(double target_heading) {
    PID turn_PID(3, 0.05, 13.0, 15);

    while (!turn_PID.is_settled(100)) {
        std::uint32_t start = pros::millis();

        double current_heading = robot.get_heading();
        double error = target_heading - current_heading;

        if (error > 180) {
            error -= 360;
        }
        else if (error <= -180) {
            error += 360;
        }

        double power = turn_PID.compute(error, 1);
        
        set_drive_voltages(0, power);
        pros::Task::delay_until(&start, 10);
    }
    set_drive_voltages(0, 0);
}

void swing_right_to_heading(double target_heading) {
    PID turn_PID(3, 0.05, 13.0, 15);

    while (!turn_PID.is_settled(100)) {
        std::uint32_t start = pros::millis();

        double current_heading = robot.get_heading();
        double error = target_heading - current_heading;

        if (error > 180) {
            error -= 360;
        }
        else if (error <= -180) {
            error += 360;
        }

        double power = turn_PID.compute(error, 1);
        
        set_drive_voltages(-power, 0);
        pros::Task::delay_until(&start, 10);
    }
    set_drive_voltages(0, 0);
}

void turn_to_point(double x, double y) {
    PID turn_point_PID(1.83, 0.05, 15.0, 15);

    while (!turn_point_PID.is_settled(100)) {
        std::uint32_t start = pros::millis();
        double current_heading = robot.get_heading();
        double error = rad_to_deg(atan2(y - robot.get_y(), x - robot.get_x())) - current_heading;

        if (error > 180) {
            error -= 360;
        }
        else if (error <= -180) {
            error += 360;
        }

        double power = turn_point_PID.compute(error, 2);
        
        set_drive_voltages(-power, power);
        pros::Task::delay_until(&start, 10);
    }
    set_drive_voltages(0, 0);
}

void turn_and_drive_to_point(double x, double y) {
    turn_to_point(x, y);
    drive_to_point(x, y);
}