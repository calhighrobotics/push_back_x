#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "auton/autonFunctions.h"
#include "pros/rtos.hpp"
#include "velocityController.h"
#include "paths.h"
#include "ramsete.h"


    const VelocityControllerConfig config{
    12.4370890785,
    0.803031225567,
    0.664537661342,
    0.472796490892,
    0.236548087393,
    25.2621164319,
    524.703492373,
};
    
    RamsetePathFollower ramsete(config, 2, 0.7);
    


void precompute_auton_paths() {
    std::vector<std::string> paths = {right_1, right_2, left_1, skills_1};
    ramsete.precompute_paths(paths);
}

void right_auton()
{

    const int triball_delay = 500;
    const int dual_ball_delay = 500;
    const int match_load_delay = 500;

    chassis.setPose(-48.147, -10.576, 115);
    intake(12000);
    chassis.turnToPoint(-23.378, -21.986, 1000);
    chassis.moveToPoint(-23.378, -21.986, 1000);
    pros::delay(triball_delay);
    //Ramsete path to dual_ball
    ramsete.followPath(right_1, {.path_index = 1});

    pros::delay(dual_ball_delay);

    //Ramsete backward path to -47, -47
    ramsete.followPath(right_2, {.backwards = true, .path_index = 2});

    intake_stop();
    chassis.turnToPoint(-58, -47, 1000);
    matchload_state(true);
    chassis.moveToPoint(-58, -47, 1000);
    intake();
    //match_load_wiggle(match_load_delay);
    pros::delay(match_load_delay);
    matchload_state(false);
    chassis.moveToPoint(-31, -47, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
}

void carry_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,12,2000);
}

void left_auton() {

    const int match_load_delay = 500;
    const int dual_ball_delay = 500;
    const int midgoal_delay = 1000;

    chassis.setPose(-48.147, 10.854, 75);
    intake();
    ramsete.followPath(left_1, {.path_index = 3});

    pros::delay(dual_ball_delay);

    chassis.turnToPoint(1.392, 64.289, 500);
    chassis.moveToPoint(-16.977, 17.255, 1000, {.forwards = false});

    //Score on midgoal
    chassis.turnToPoint(-36.736, 39.519, 500);
    chassis.moveToPoint(-11.689, 11.689, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake_stop();
    score_midgoal(6000);

    pros::delay(midgoal_delay);

    chassis.turnToPoint(-40.354, 47.034, 500);
    chassis.moveToPoint(-40.354, 47.034, 1000);

    //Intake from matchload
    chassis.turnToPoint(-58.723, 46.755, 500);
    chassis.waitUntilDone();
    matchload_state(true);
    chassis.moveToPoint(-58.723, 46.755, 1000);
    chassis.waitUntilDone();
    intake();
    pros::delay(match_load_delay);
    //match_load_wiggle(300);

    //Score on longgoal

    chassis.moveToPoint(-30.057,  47.034, 1000, {.forwards = false});
    pros::delay(100);
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
}

void elim_auton() {}

void awp_auton() {

    const int longgoal_delay = 1000;
    const int midgoal_delay = 1000;
    const int matchload_delay = 1000;

    chassis.setPose(-51.208, -15.307, 180);
    chassis.moveToPoint(-51.487, -47.034, 1000);
    chassis.waitUntilDone();

    chassis.turnToPoint(-58.723, -47.034, 500);
    chassis.waitUntilDone();
    matchload_state(true);
    chassis.moveToPoint(-58.723, -47.034, 1000);
    intake();
    pros::delay(matchload_delay);
    
    chassis.moveToPoint(-30.335, -47.034, 1000, {.forwards = false});
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
    pros::delay(longgoal_delay);

    chassis.swingToPoint(-22.821, -23.099, 1000);
    intake();
    chassis.moveToPoint(-22.821, -23.099, 1000);
    chassis.turnToPoint(-22.264, 21.986, 1000);
    chassis.moveToPoint(-22.264, 21.986, 1000);
    chassis.turnToPoint(-36.736, 36.458, 500);
    chassis.moveToPoint(-11.411, 11.411, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake_stop();
    score_midgoal(6000);
    pros::delay(midgoal_delay);

    chassis.moveToPoint(-48, 47, 1000);
    matchload_state(true);
    chassis.turnToPoint(-58.723, 46.755, 500);
    chassis.moveToPoint(-58.723, 46.755, 300);
    intake();
    chassis.waitUntilDone();
    pros::delay(matchload_delay);

    chassis.moveToPoint(-28.387, 47.034, 1000, {.forwards = false});
    chassis.waitUntil(10);
    matchload_state(false);
    chassis.waitUntilDone();

    intake_stop();
    score_longgoal();

}

void skills_auton() {

    const int match_load_delay = 1000;
    const int midgoal_delay = 1000;
    const int longgoal_delay = 1500;

    chassis.setPose(-53.991, 15.307, 0);
    intake();
    chassis.moveToPoint(-53.713, 47.034, 1000);
    chassis.turnToPoint(-60.114, 47.034, 500);
    chassis.waitUntilDone();
    matchload_state(true);
    pros::delay(match_load_delay);

    chassis.moveToPoint(-28.944, 47.034, 1000, {.forwards = false});
    chassis.waitUntilDone();
    matchload_state(false);
    intake_stop();
    score_longgoal();
    pros::delay(longgoal_delay);

    chassis.moveToPoint(-41.189, 47.034, 500);
    chassis.turnToHeading(53, 500);
    chassis.waitUntilDone();
    ramsete.followPath(skills_1, {.path_index = 4});

    matchload_state(true);
    chassis.turnToPoint(59, 47, 500);
    chassis.moveToPoint(59, 47, 500);
    intake();
    chassis.waitUntilDone();
    pros::delay(match_load_delay);

    chassis.moveToPoint(29.5, 47, 1000, {.forwards = false});
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
    pros::delay(longgoal_delay);
}