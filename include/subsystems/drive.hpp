#include "main.h"

void set_drive_voltages(double left_voltage, double right_voltage);
void split_arcade();
void print_information();
void drive_for(double distance);
void drive_to_point(double x, double y);
void turn_to_heading(double heading);
void swing_left_to_heading(double heading);
void swing_right_to_heading(double heading);
void turn_to_point(double x, double y);
void turn_and_drive_to_point(double x, double y);