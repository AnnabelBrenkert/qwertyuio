#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "pros/adi.hpp"

extern Drive chassis;

extern pros::Motor fly;
extern pros::Motor arm;
extern pros::Controller master;
extern pros::ADIDigitalOut wings;
extern pros::ADIDigitalOut hang;

void Match_Auton();
void Skills1();
void Skills2();
void wait_until_change_speed();
void swing_example();
void combining_movements();
void interfered_example();

void default_constants();
void one_mogo_constants();
void two_mogo_constants();
void exit_condition_defaults();
void modified_exit_condition();