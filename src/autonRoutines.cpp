#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "autonFunctions.h"

void right_auton()
{
    chassis.setPose(0,0,25.796);

    //First 3 ball intaking
    intake();
    chassis.turnToPoint(13.5,26.9,1000);
    chassis.moveToPoint(7.8, 16.6, 2000, {.maxSpeed = 100, .minSpeed = 20});
    chassis.turnToPoint(12.6, 26.6, 1000);
    chassis.moveToPoint(12.6, 26.6, 2000, {.maxSpeed = 30});
    pros::delay(2500);
    intake_stop();
    
    chassis.moveToPoint(10, 24, 2000, {.forwards = false});
    //Outtake blocks into center
    chassis.turnToHeading(-45, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0,17, 2000);
    chassis.waitUntilDone();
    score_bottomgoal();
    pros::delay(3000);
    intake_stop();

    //Matchload and Long goal Scoring
    chassis.moveToPoint(0, -33, 3000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntilDone();
    aligner.retract();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.turnToPoint(-10,-3.4, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    matchload_prep();
    pros::delay(500);
    chassis.moveToPoint(0,16, 1000, {.minSpeed = 100});
    intake();
    pros::delay(2500);
    intake_stop();
    
    chassis.moveToPoint(0,0,1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    longgoal_prep();
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,15,1000);
    score_longgoal();
    pros::delay(10000);
    


    /*
    chassis.turnToPoint(34.6, 29.9, 1000);
    matchload_prep();
    intake();
    pros::delay(2000);
    intake_stop();
    chassis.moveToPoint(34.6, 29.9, 1000);
    chassis.moveToPoint(34.6, 22.5, 1500, {.forwards = false, .maxSpeed = 75});
    longgoal_prep();
    chassis.turnToPoint(34.6, 50, 1000);
    chassis.moveToPoint(34.6, 40, 1000, {.forwards = true, .maxSpeed = 75});
    score_longgoal();
    pros::delay(3000);
    intake_stop();
    */
}

void left_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,12,2000);
}

void carry_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,12,2000);
}

void elim_auton() {}

void awp_auton() {}