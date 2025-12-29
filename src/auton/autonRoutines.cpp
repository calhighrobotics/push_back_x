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
#include "distanceReset.h"
#include "MCL.h"


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
    score_midgoal(12000);

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

    const int longgoal_delay = 2000;
    const int midgoal_delay = 3000;
    const int matchload_delay = 1500;
    const int mid_triball_delay = 500;
    const int longgoal_offset = 6;
    const int midgoal_offset = 12;
    const int matchload_offset = 12.9;
    
    MCL::StartMCL(-51.25, -18.5, 180);
    pros::Task mclTask(MCL::MonteCarlo);
    enable_fused_odometry(true);
    chassis.setPose(-51.25, -18.5, 180);
    chassis.moveToPoint(-51.25, -48, 1500);
    chassis.waitUntilDone();

    chassis.turnToPoint(-72, -48, 750);
    matchload_state(true);
    chassis.waitUntilDone();
    //distanceReset();
    pros::delay(500);
    intake();
    chassis.moveToPoint(-72 + matchload_offset, -48, 1500);
    chassis.waitUntilDone();
    matchload_wiggle(matchload_delay);

    chassis.moveToPoint(-24 - longgoal_offset, -48, 1000, {.forwards = false});
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
    pros::delay(longgoal_delay);
    intake_stop();

    chassis.swingToPoint(-24 , -24, lemlib::DriveSide::RIGHT, 750);
    intake();
    chassis.moveToPoint(-24 , -24, 1000);
    chassis.waitUntilDone();
    pros::delay(mid_triball_delay);
    /*
    chassis.turnToPoint(-24, 24, 1000);
    chassis.moveToPoint(-24, 24, 2000);
    chassis.waitUntil(5);
    matchload_state(false);
    chassis.waitUntilDone();
    matchload_state(true);
    pros::delay(mid_triball_delay);
    chassis.turnToPoint(-48, 48, 1000);
    chassis.waitUntilDone();
    matchload_state(false);
    chassis.moveToPoint(0 - midgoal_offset, 0 + midgoal_offset, 1500, {.forwards = false});
    chassis.waitUntilDone();
    intake_stop();
    score_midgoal(6000);
    pros::delay(midgoal_delay);

    chassis.moveToPoint(-48, 48, 1500);
    matchload_state(true);
    chassis.turnToPoint(-72, 48, 1000);
    chassis.moveToPoint(-72 + matchload_offset, 48, 1500);
    intake();
    chassis.waitUntilDone();
    pros::delay(matchload_delay);

    chassis.moveToPoint(-24 - longgoal_offset, 48, 2000, {.forwards = false});
    chassis.waitUntil(5);
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
    */
}

void skills_auton() {

    const int match_load_delay = 1000;
    const int midgoal_delay = 1000;
    const int longgoal_delay = 1500;

    chassis.setPose(-49.25,-18.5,180);
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