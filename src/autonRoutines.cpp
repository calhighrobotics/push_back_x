#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"

void red_right()
{
    intakeMotor.move_voltage(12000);
    chassis.setPose(-51.322, -10.166, 110);
    pros::delay(100);
    chassis.moveToPoint(-26.5, -18.5, 5000);
    chassis.turnToHeading(-11,-19);
    pros::delay(100);
    chassis.moveToPoint(-10, -15, 5000);
    intakeMotor.move_voltage(-12000);
    pros::delay(2000);
    chassis.moveToPoint(-47.61, -48.34, 5000, {.forwards = false});
    chassis.turnToPoint(-36.5, -47.5, 1000);
    pros::delay(100);
    intakeMotor.move_voltage(12000);
    chassis.moveToPoint(-36.5, -47.5, 5000);
    pros::delay(100);
    chassis.moveToPoint(-35, -35, 5000, {.forwards = false});
    intakeMotor.brake();
    matchload_mech.extend();
    chassis.turnToPoint(-38, -47.6, 1000);
    pros::delay(100);
    chassis.moveToPoint(-31, -47.6, 5000);
    intakeMotor.move_voltage(12000);
    pros::delay(2000);
    chassis.moveToPoint(-10, -49.3, 5000, {.forwards = false});
    pros::delay(100);
    intakeMotor.brake();
    chassis.turnToPoint(0, -49.3, 1000);
    
}