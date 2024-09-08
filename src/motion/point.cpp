#include "main.h"
#include "motion/point.hpp"

Point::Point(double new_x, double new_y) {
    x = new_x;
    y = new_y;
}

Point::Point(double new_x, double new_y, double new_heading) {
    x = new_x;
    y = new_y;
    heading = new_heading;
}

void Point::set_x(double new_x) {
    x = new_x;
}

void Point::set_y(double new_y) {
    y = new_y;
}

void Point::set_heading(double new_heading) {
    heading = new_heading;
}

double Point::get_x() {
    return x;
}

double Point::get_y() {
    return y;
}

double Point::get_heading() {
    return heading;
}