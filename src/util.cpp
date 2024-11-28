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
 * Makes an angle between 0 and 360 degrees while keeping the same value mod 360.
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
 * Makes an angle between -180 and 180 degrees while keeping the same value mod 360.
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
 * Returns whether a number is positive or negative.
*/
double sign(double num) {
    if (num >= 0) {
        return 1;
    }
    else {
        return -1;
    }
}

/**
 * Calculates the distance between two points using the Pythagorean Theorem
*/
double distance_between_points(Point first, Point second) {
    return sqrt(pow(second.get_x() - first.get_x(), 2) + pow(second.get_y() - first.get_y(), 2));
}