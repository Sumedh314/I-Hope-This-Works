#include "main.h"
#include "point.hpp"

// Pi
const double pi = 3.14159265358979;

double deg_to_rad(double angle);
double rad_to_deg(double angle);
double reduce_0_to_360(double angle);
double reduce_negative_180_to_180(double angle);
double sign(double num);
double clamp(double value, double max_range);
double clamp(double value, double min_value, double max_value);
double distance_between_points(Point first, Point second);