#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "autonFunctions.h"

void right_auton()
{
    double start_time = pros::millis();
    chassis.setPose(0,0,25.796);

    //First 3 ball intaking
    intake();
    chassis.turnToPoint(13.5,26.9,1000);
    chassis.moveToPoint(7.8+0.5, 16.6+0.5, 2000, {.maxSpeed = 100, .minSpeed = 20});
    chassis.turnToPoint(12.6, 26.6, 1000);
    chassis.moveToPoint(12.6, 26.6, 2000, {.maxSpeed = 30});
    pros::delay(2000);
    intake_stop();
    
    chassis.moveToPoint(10, 24, 2000, {.forwards = false});
    //Outtake blocks into center
    chassis.turnToHeading(-41, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0,13.7, 2000);
    chassis.waitUntilDone();
    score_bottomgoal();
    pros::delay(2000);
    intake_stop();

    //Matchload
    chassis.moveToPoint(0, -35.5, 3000, {.forwards = false, .maxSpeed = 100});
    aligner.retract();
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.turnToPoint(-10,-3.7, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    matchload_prep();
    chassis.moveToPoint(-0.5,17, 1000, {.minSpeed = 100});
    intake();
    pros::delay(2000);
    intake_stop();
    
    //Long goal Scoring
    chassis.moveToPoint(0,0,1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(195, 1000);
    longgoal_prep();
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.moveToPoint(1.5,17,1000);
    score_longgoal();
    pros::delay(2500);
    chassis.moveToPoint(0,0,10000, {.forwards = false});
    
    controller.print(0,0,"Time: %f", (pros::millis() - start_time)/1000);
}

void left_auton() {
    /*
    chassis.setPose(0,0,-25.796);
    intake();
    chassis.turnToPoint(-13.5,26.9,1000);
    chassis.moveToPoint(-7.8+0.5, 16.6+0.5, 2000, {.maxSpeed = 100, .minSpeed = 20});
    chassis.turnToPoint(-12.6, 26.6, 1000);
    chassis.moveToPoint(-12.6, 26.6, 2000, {.maxSpeed = 30});
    pros::delay(2500);
    intake_stop();
    
    chassis.moveToPoint(-10, 24, 2000, {.forwards = false});
    chassis.turnToHeading(41, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0,17, 2000);
    chassis.waitUntilDone();
    score_bottomgoal();
    pros::delay(3000);
    intake_stop();
    
    chassis.setPose(0,0,-90);
    chassis.moveToPoint(15, 0, 1000);
    chassis.turnToHeading(-180, 1000);
    matchload_prep();
    chassis.moveToPoint(15, -4, 1000, {.minSpeed = 100});
    intake();
    pros::delay(2000);
    */
    

    
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,3,1000);
    chassis.waitUntilDone();
    

}

void carry_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,12,2000);
}

void elim_auton() {}

void awp_auton() {}