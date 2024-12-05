#include "main.h"
#include "util.hpp"
#include "point.hpp"

/**
 * Converts an angle from degrees to radians.
*/
double deg_to_rad(double angle) {
    return angle * pi / 180;
}

/**
 * Converts an angle from radians to degrees.
*/
double rad_to_deg(double angle) {
    return angle * 180 / pi;
}

/**
 * Makes an angle be between 0 and 360 degrees while keeping the same value mod 360.
*/
double reduce_0_to_360(double angle) {
    while (angle < 0) {
        angle += 360;
    }
    while (angle >= 360) {
        angle -= 360;
    }
    return angle;
}

/**
 * Makes an angle be between -180 and 180 degrees while keeping the same value mod 360.
*/
double reduce_negative_180_to_180(double angle) {
    while (angle < -180) {
        angle += 360;
    }
    while (angle >= 180) {
        angle -= 360;
    }
    return angle;
}

/**
 * Returns whether a number is positive or negative, or returns 0 if the number is 0.
*/
double sign(double num) {
    if (num > 0) {
        return 1;
    }
    else if (num < 0) {
        return -1;
    }
    return 0;
}

/**
 * Makes a value be within a given range where the bounds are the positive and negative value of max_range.
*/
double clamp(double value, double max_range) {
    return clamp(value, -max_range, max_range);
}

/**
 * Makes a value be within a given range.
*/
double clamp(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/**
 * Calculates the distance between two points using the Pythagorean Theorem.
*/
double distance_between_points(Point first, Point second) {
    return sqrt(pow(second.get_x() - first.get_x(), 2) + pow(second.get_y() - first.get_y(), 2));
}