#include "main.h"
#include "robot_subsystems/robot-config.hpp"

void toggle_loader() {
    bool deployed = false;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            match_loader.set_value(!deployed);
            deployed = !deployed;
        }
        pros::delay(20);
    }
}

void toggle_chute(){
    bool deployed = false;
    while (true) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        chute.set_value(!deployed);
        deployed = !deployed;
    }
    pros::delay(20);
}
}

void toggle_descore(){
    bool deployed = false;
    while (true) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        descore.set_value(!deployed);
        deployed = !deployed;
    }
    pros::delay(20);
}
}


void deploy_chute(){
    chute.set_value(HIGH);
}

void undeploy_chute(){
    chute.set_value(LOW);
}


void deploy_loader() {
    match_loader.set_value(HIGH);
}

void undeploy_loader() {
    match_loader.set_value(LOW);
}

void deploy_descore(){
    descore.set_value(HIGH);
}
void undeploy_descore(){
    descore.set_value(LOW);
}