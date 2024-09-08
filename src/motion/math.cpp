#include "main.h"
#include "motion/math.hpp"
#include "motion/point.hpp"

double rad_to_deg(double angle) {
    return angle * 180 / pi;
}

double deg_to_rad(double angle) {
    return angle * pi / 180;
}

double distance_between_points(Point point1, Point point2) {
    double distance = sqrt(pow(fabs(point2.get_x() - point1.get_x()), 2) + pow(fabs(point2.get_y() - point1.get_y()), 2));
    return distance;
}