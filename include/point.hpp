#include "main.h"

class Point {
    double x = 0;
    double y = 0;

    public:
        Point(double x, double y);

        void set_x(double new_x);
        void set_y(double new_y);

        double get_x();
        double get_y();
};