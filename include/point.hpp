#ifndef POINT_HPP
#define POINT_HPP
#include "main.h"

class Point {
    protected:
        double x = 0;
        double y = 0;

    public:
        // Point();
        Point(double x, double y);

        void set_x(double x);
        void set_y(double y);
        void set_coordinates(double x, double y);

        double get_x();
        double get_y();
};
#endif