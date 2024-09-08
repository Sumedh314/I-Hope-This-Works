#ifndef POINT_H
#define POINT_H
#include "main.h"

class Point {
    private:
        double x = 0;
        double y = 0;
        double heading = 0;
        
    public:
        Point(double new_x, double new_y);
        Point(double new_x, double new_y, double new_heading);

        void set_x(double new_x);
        void set_y(double new_y);
        void set_heading(double new_heading);

        double get_x();
        double get_y();
        double get_heading();
};

extern Point robot;

#endif