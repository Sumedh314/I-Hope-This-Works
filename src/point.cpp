#include "main.h"
#include "point.hpp"

/**
 * Creates a new Point object with no default coordinates.
*/
Point::Point() {}

/**
 * Creates a new Point object with default coordinates.
*/
Point::Point(double x, double y) :
    x(x),
    y(y)
{}

/**
 * Sets the x value for this point.
*/
void Point::set_x(double new_x) {
    x = new_x;
}

/**
 * Sets the y value for this point.
*/
void Point::set_y(double new_y) {
    y = new_y;
}

/**
 * Sets the coordinates of the robot.
*/
void Point::set_coordinates(double new_x, double new_y) {
    x = new_x;
    y = new_y;
}

/**
 * Returns the x value for this point.
*/
double Point::get_x() {
    return x;
}

/**
 * Returns the y value for this point.
*/
double Point::get_y() {
    return y;
}