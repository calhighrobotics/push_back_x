#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "autonFunctions.h"

void right_auton()
{
    chassis.setPose(0,0,0);

    //First 3 ball intaking
    pros::delay(100);
    intake();
    chassis.moveToPoint(0, 25.25, 2000, {.maxSpeed = 30});
    chassis.moveToPoint(0, 45, 2000, {.maxSpeed = 30});
    pros::delay(1500);
    intake_stop();
    
    //Outtake blocks into center
    chassis.moveToPoint(0.62, 29.33, 500, {.forwards = false});
    chassis.turnToPoint(-2.5, 41.5, 1000);
    chassis.moveToPoint(-2.5, 41.5, 2000, {.maxSpeed = 50});
    pros::delay(1000);
    score_bottomgoal();
    pros::delay(3000);
    intake_stop();

    //Matchload and Long goal
    chassis.moveToPoint(42.15, 22.5, 3000, {.forwards = false, .maxSpeed = 100});
    aligner.retract();
    chassis.turnToPoint(34.6, 29.9, 1000);
    matchload_mech.extend();
    pros::delay(200);
    chassis.moveToPoint(34.6, 29.9, 1000);


}

void left_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,12,2000);
}

void carry_auton() {
    chassis.moveToPoint(0,12,2000);
}