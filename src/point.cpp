#include "main.h"
#include "point.hpp"

/**
 * Creates a new Point object with no default coordinates.
*/
// Point::Point() {}

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
void Point::set_x(double x) {
    this->x = x;
}

/**
 * Sets the y value for this point.
*/
void Point::set_y(double y) {
    this->y = y;
}

/**
 * Sets the coordinates of the robot.
*/
void Point::set_coordinates(double x, double y) {
    this->x = x;
    this->y = y;
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