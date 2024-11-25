#ifndef POINT_HPP
#define POINT_HPP
#include "main.h"

class Point {
    double x = 0;
    double y = 0;

    public:
        Point();
        Point(double x, double y);

        void set_x(double new_x);
        void set_y(double new_y);
        void set_coordinates(double new_x, double new_y);

        double get_x();
        double get_y();
};
#endif