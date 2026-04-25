#include "IntakeAntiJam.h"
#include "globals.h" 
#include "auton/autonFunctions.h"
#include "lemlib/chassis/chassis.hpp"
#include "ltv.h"
#include "pros/rtos.hpp"
#include "velocityController.h"
#include "paths.h"
#include "distanceReset.h"
#include "colorSort.h"
#include "crossBarrierDetection.h"
#include <cstdlib>
#include <map>
#include <sys/types.h>
#include "RCL.h"

/*

Straight:
KV = 5.83366315659
1.14915679465
0.277890461166
11.8794542097
59.3640060487

Turn:
6.69921571762
1.01377617649
0.956265367397
14.8634454339
101.977995601

*/

const int longgoal_delay = 1100;
const int midgoal_delay = 1000;
const int matchload_delay = 500;
const int mid_triball_delay = 500;
const int longgoal_offset = 6;
const int midgoal_offset = 11.7;
const int matchload_offset = 11.9;
const int triball_delay = 500;
const int dual_ball_delay = 500;

const VelocityControllerConfig config{
5.09498070723,
0.23396112183,
1.22496514644,
0.964796393603,
0.522291806997,
10.933332351,
44.6788428062,
};

LTVPathFollower ltv(config);



void precompute_auton_paths(std::string path_name) {
    std::map<std::string, std::vector<std::string>> pathMap = 
    {
        {"Skills", {"skills_1", "skills_2", "skills_3", "skills_4", "parkingzone_curve"}},
        {"Right 7 Split", {"awp_1"}},
        {"Right 7", {"right_7_1"}},
        {"AWP", {"awp_2"}},
    };
    for (const auto& [key, value] : pathMap) {
    {
        if(path_name == key)
        {
            ltv.precompute_paths(pathMap[key]);
            return;
        }
    }
}


}

void test_auton()
{   
    chassis.setPose(62.5, -33, 180);
    chassis.setPose(62.5, -33, chassis.getPose().theta);
    distanceReset(true, false);
    rcl.start();
    chassis.turnToHeading(240, 2000, {}, false);
    distanceReset(true, false);
    rcl.start();
    chassis.moveToPoint(45, -47, 2000, {});
    chassis.turnToHeading(90, 2000);
    chassis.moveToPose(27, -47, 270, 2000, {}, false);
    score_longgoal_auton(600, allianceColor, 1000);
    intake();
    chassis.moveToPose(56.5, 47, 90, 2000, {.minSpeed = 30}, false);
    chassis.tank(0.35, 0, config, matchload_delay);
    distanceReset(true);

    rcl.start();
    chassis.moveToPoint(50, chassis.getPose().y, 1500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 8});
    intake_stop();
    intakeMotor.move_velocity(200);
    chassis.moveToPose(24,-61, 90, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 7});
    intake_stop();
    matchload_state(false);
    chassis.moveToPoint(-29, -58.5, 1500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 8});
    rcl.end();
    chassis.moveToPose(-45, -45, 130, 1500, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(270, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    distanceReset(true);
    chassis.moveToPoint(-27, -46, 1500, {.forwards = false, .minSpeed = 40}, false);
    chassis.tank(-0.5, 0, config, 150);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    distanceReset(true, false);

    //PARKING ZONE
    matchload_state(true);
    intake();
    distanceReset(true, false);
    rcl.start();
    chassis.turnToPoint(-56.5, -46.5, 1200, { .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(-56.5, -46.5, 90, 1200, {.minSpeed = 30}, false);
    int collision_timeout = 0;
    while(chassis.isInMotion())
    {
        if(chassis.detect_collision())
        {
            collision_timeout += 10;
        }
        if(collision_timeout > 100)
        {
            chassis.cancelMotion();
        }
        pros::delay(10);
    }
    chassis.tank(0.3, 0, config, matchload_delay);
    chassis.brake();
    distanceReset(true);
    chassis.turnToPoint(-27, -47.5, 1200, {.forwards = false, .minSpeed = 35, .earlyExitRange = 25});
    intake_stop();
    intakeMotor.move_velocity(200);
    chassis.moveToPose(-27, -47.5, 270, 1200, {.forwards = false, .minSpeed = 20}, false);
    intake_stop();
    jamManager.enable_anti_jam(true);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    rcl.end();
    matchload_state(false);
    distanceReset(true);
    ltv.followPath(parkingzone_curve2, {.q_x = 4.5, .q_y = 550, .q_theta = 60});
    ltv.waitUntil(4.5);
    score_midgoal();
    ltv.waitUntil(9);
    intake_stop();
    intake();
    ltv.waitUntilDone();
    intake();
    chassis.tank(1.5, -1, config, 500);
    chassis.brake();
    rcl.end();
}

void left_rush()
{
    const float matchload_delay = 380;
    descore.extend();
    rcl.start();
    chassis.setPose(-51.6, 16.5, 90);
    chassis.moveToPoint(-26.8, 21.8, 2000, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(10);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-36, 42.5, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 15});
    chassis.moveToPose(-36, 42.5, 140, 1000, {.forwards = false, .minSpeed = 75, .earlyExitRange = 8});
    matchload_state(false);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 200, {.minSpeed = 127, .earlyExitRange = 20});
    chassis.turnToHeading(270, 2000, {.minSpeed = 127});
    chassis.moveToPoint(-27, 48, 1500, {.forwards = false}, false);
    //chassis.tank(-1, 0, config, 100);
    score_longgoal_auton(600, allianceColor, 1000);
    distanceReset(false, true, false, false, true);
    chassis.moveToPose(-37, 36.6, 240, 1500, {.minSpeed = 70, .earlyExitRange = 9});
    chassis.turnToPoint(-2.5, 37.25, 1000, {.forwards = false, .minSpeed = 55, .earlyExitRange = 15});
    distanceReset(true);
    chassis.moveToPose(-2.5, 37.25, 270, 1500, {.forwards = false, .minSpeed = 20});
    chassis.waitUntil(5.5);
    descore.retract();
    chassis.waitUntilDone();
    rcl.end();
    chassis.brake();
    chassis.turnToHeading(285, 1000, {}, false);
}


void right_auton_split()
{
    const float matchload_delay = 380;
    descore.extend();
    rcl.start();
    jamManager.enable_anti_jam(false);
    chassis.setPose(-49.7, -14, 180);
    chassis.moveToPoint(-49.7, -45, 1500, {});
    chassis.waitUntil(5);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-69, -47, 1500, {}, false);
    distanceReset(true);
    intake();
    chassis.moveToPose(-56.5, -47, 270, 1000, {.lead = 0.2, .minSpeed = 50}, false);
    chassis.waitUntilDone();
    chassis.tank(0.7, 0, config, 120);
    distanceReset(true);
    chassis.moveToPose(-28, -48, 270, 1500, {.forwards = false, .minSpeed = 60, .earlyExitRange = 5}, false);
    score_longgoal_auton(600, allianceColor,1000);
    distanceReset(true);
    matchload_state(false);
    rcl.end();
    chassis.turnToPoint(-27, -22, 900, {}, false);
    distanceReset(true, false, false, false, true);
    intake();
    chassis.moveToPoint(-24, -20, 2000, {.minSpeed = 55, .earlyExitRange = 8});
    chassis.waitUntil(11);
    matchload_state(true);
    chassis.waitUntilDone();
    distanceReset(true);
    matchload_state(false);
    pros::delay(100);
    chassis.turnToPoint(-10, -8, 1200, {.minSpeed = 35, .earlyExitRange = 10});
    chassis.moveToPose(-10, -8, 43, 1300, {.lead = 0.3});
    matchload_state(false);
    intake();
    chassis.waitUntil(12);
    matchload_state(false);
    chassis.waitUntilDone();
    outtake();
    pros::delay(1500);
    intake_stop();
    rcl.start();
    chassis.moveToPoint(-39, -35, 2000, {.forwards = false}, false);
    chassis.turnToPoint(-14, -36, 2000, {.minSpeed = 45, .earlyExitRange = 30}, false);
    distanceReset(true, true, 7);
    chassis.moveToPose(-11.5, -36, 90,2000, {.lead = 0.2, .minSpeed = 20});
    chassis.waitUntil(5.5);
    descore.retract();
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.turnToHeading(60, 2000, {.minSpeed = 35}, false);
    rcl.end();

}

void right_rush()
{
    descore.extend();
    chassis.setPose(-51.6, -16.5, 90);
    chassis.moveToPoint(-26.5, -21.8, 2000,  {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(11);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-36, -40, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(-36, -40, 20, 1000,  {.forwards = false, .minSpeed = 70, .earlyExitRange = 8});
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1500, {.minSpeed = 127, .earlyExitRange = 30});
    chassis.turnToHeading(270, 1000, {.minSpeed = 127, .earlyExitRange = 5}, false);
    chassis.moveToPoint(-27, -48, 1300, {.forwards = false, .minSpeed = 20}, false);
    score_longgoal_auton(600, allianceColor, 1000);
    distanceReset(true);
    matchload_state(false);
    chassis.moveToPose(-46, -59.5, 260, 1000, {.minSpeed = 60, .earlyExitRange = 7});
    chassis.moveToPose(-4.5, -58.5, 270, 2000,  {.forwards = false});
    chassis.waitUntil(4);
    descore.retract();
    chassis.turnToHeading(290, 1000);
    
    descore.retract();
}   

void right_7_wing()
{
    descore.extend();
    chassis.setPose(-47, -16.5, 90);
    ltv.followPath(right_7_1, {.q_x = 6.5, .q_y = 100, .q_theta = 100, .r_ang = 0.25, .r_vel = 1});
    intake();
    ltv.waitUntil(19);
    matchload_state(true);
    ltv.waitUntilDone();
    chassis.tank(0.55, 0, config, 410);
    distanceReset(true);
    rcl.start();
    chassis.turnToPoint(-24, -47, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 10});
    chassis.moveToPoint(-24, -47, 1300, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    score_longgoal_auton(600, allianceColor,1500);
    distanceReset(true);
    matchload_state(false);
    chassis.moveToPose(-34, -60, 240, 1000, {.minSpeed = 60, .earlyExitRange = 9});
    chassis.turnToPoint(-1.2, -59, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 30});
    distanceReset(true, true, 3.5);
    rcl.end();
    chassis.moveToPoint(-1.2, -58.6, 2000, {.forwards = false, .minSpeed = 40});
    chassis.waitUntil(4);
    descore.retract();
    chassis.waitUntilDone();
    chassis.brake();
    rcl.end();
}

void carry_auton() {
    descore.extend();
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 6, 10000);
}

void left_auton_split() {
    const float starting_roll = imu.get_roll();
    const float matchload_delay = 380;
    descore.extend();
    rcl.start();
    chassis.setPose(-51.6, 16.5, 90);
    chassis.moveToPoint(-20, 25, 2000, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(10);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-37.5, 42.5, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 15});
    chassis.moveToPose(-37.5, 42.5, 140, 1000, {.forwards = false, .minSpeed = 75, .earlyExitRange = 8});
    matchload_state(false);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 200, {.minSpeed = 127, .earlyExitRange = 20});
    chassis.turnToHeading(270, 2000, {.minSpeed = 127});
    chassis.moveToPoint(-27, 48, 600, {.forwards = false}, false);
    chassis.tank(-1, 0, config, 100);
    score_longgoal_auton(600, allianceColor, 1200);
    distanceReset(false, true, false, false, true);
    chassis.turnToPoint(-58, 47, 2000, {.minSpeed = 35, .earlyExitRange = 30}, false);
    matchload_state(true);
    chassis.moveToPose(-58.5, 47, 270, 1500, {.lead = 0.2, .minSpeed = 20});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntilDone();
    chassis.tank(35, 35);
    pros::delay(600);
    distanceReset(true, false);
    chassis.tank(-2.0, 0, config, 150);
    chassis.moveToPose(-10, 11, 315, 2000,  {.forwards = false, .minSpeed = 65}, false);
    chassis.tank(-35, -35);
    pros::delay(100);
    chassis.tank(-20, -20);
    score_midgoal();
    pros::delay(1000);
    chassis.brake();
    rcl.end();
    relativeMotion(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, 10, 1000, true, 0);
    mid_descore.extend();
    relativeMotion(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, -8.5, 1000, false, 0);
    intake_stop();
    chassis.moveToPoint(-36, 37, 2000, {});
    matchload_state(false);
    chassis.turnToPoint(-10, 37.5, 2000, {.forwards = false, .minSpeed = 45, .earlyExitRange = 6});
    distanceReset(true, true, 5);
    chassis.moveToPose(-7, 37.5, 270,2000, {.forwards = false, .lead = 0.5});
    chassis.waitUntil(6);
    descore.retract();
    chassis.waitUntilDone();
}


void awp_auton() {
    int val = 0;
    descore.extend();
    jamManager.enable_anti_jam(false);
    chassis.setPose(-49.7, -14, 180);
    chassis.moveToPoint(-49.7, -45, 1500, {});
    chassis.waitUntil(5);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-69, -47, 1500, {}, false);
    distanceReset(true);
    intake();
    chassis.moveToPoint(-55.5, -47, 1000, {.minSpeed = 50}, false);
    chassis.waitUntilDone();
    chassis.tank(0.7, 0, config, 300);
    distanceReset(true);
    chassis.moveToPose(-28, -46.7, 270, 1500, {.forwards = false, .minSpeed = 70, .earlyExitRange = 5}, false);
    score_longgoal_auton(600, Color::NONE,800);
    score_longgoal(600);
    matchload_state(false);
    chassis.turnToPoint(-24, -22, 2000,  {.minSpeed = 127, .earlyExitRange = 10}, false);
    intake_stop();
    distanceReset(false, true, false, false, true);
    ltv.followPath(awp_2, {.q_x = 7, .q_y = 450, .q_theta = 190, .r_ang = 0.2, .r_vel = 1.35});
    intake();
    ltv.waitUntil(5);
    matchload_state(true);
    ltv.waitUntil(13);
    matchload_state(false);
    ltv.waitUntil(46.5);
    matchload_state(true);
    ltv.waitUntil(62.5);
    matchload_state(false);
    ltv.waitUntilDone();
    rcl.start();
    distanceReset(false, false, true, false, true);
    chassis.turnToPoint(-27.5, 48, 1200, {.forwards = false, .minSpeed = 70, .earlyExitRange = 30});
    distanceReset(true, true, 2);
    chassis.moveToPoint(-27.5, 48, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 7}, false);
    matchload_state(true);
    jamManager.set_velocities(-200, 0, -600);
    pros::delay(100);
    score_longgoal_auton(600, Color::NONE, 1200);
    chassis.setPose(-27, 48, chassis.getPose().theta);
    distanceReset(false, true, false, false, true);
    matchload_state(true);
    intake();
    distanceReset(true, true, 5.5);
    rcl.start();
    chassis.moveToPose(-57, 46, 270, 1600, {.lead = 0.3, .minSpeed = 40}, false);
    chassis.tank(0.46, 0, config, 550);
    distanceReset(true, true,3.5);
    chassis.tank(-2.0, 0, config, 150);
    chassis.moveToPose(-10, 10, 320, 1300, {.forwards = false, .minSpeed = 75, .earlyExitRange = 3.5}, false);
    score_midgoal();
    chassis.tank(-20, -20);
    pros::delay(10000);
    rcl.end();
    
    
}

void skills_auton() {


    descore.extend();
    color_sort_enable = false;
    int start;
    int var;
    distancePose pose;

    const int longgoal_delay = 2000;
    const int midgoal_delay = 2000;
    const int lowgoal_delay = 2000;
    const int matchload_delay = 2000;
    const int mid_triball_delay = 500;
    const int longgoal_offset = 7;
    const int midgoal_offset = 11.7;
    const int matchload_offset = 11.4 + 0.7;
    const int triball_delay = 500;
    const int dual_ball_delay = 500;
    
    //LOWGOAL

    intake_lift.retract();
    chassis.setPose(-50, 0, 270);
    jamManager.enable_anti_jam(false);
    intake();
    pros::delay(900);
    chassis.tank(1.62, 0, config, 200);
    
    chassis.brake();
    chassis.tank(0.3, 0, config, 150);
    
    pros::delay(1000);
    chassis.tank(-0.2, 0, config, 400);
    
    chassis.brake();
    pros::delay(800);
    intake_stop();
    chassis.tank(-1.2, 0, config, 600);
    

    chassis.brake();
    chassis.turnToHeading(270, 2000, {}, false);
    
    chassis.setPose(-44.6, 0, chassis.getPose().theta);
    //chassis.setPose(-44.6, 0, 270);
    distanceReset(true, false);
    chassis.moveToPoint(-40, 0, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 7});
    
    jamManager.enable_anti_jam(false);
    chassis.turnToPoint(-25, -24, 1000, {}, false);
    
    intakeMotor.move_velocity(600);
    chassis.moveToPoint(-25, -24, 1500, {}, false);
    
    jamManager.set_velocities(600, -200, 600);
    chassis.turnToHeading(46.7, 1500, {}, false);
    
    pros::delay(200);
    intake_stop();
    //chassis.moveToPoint(-12, -15 - 0.85,  1500, {}, false);
    relativeMotion(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, 12.5, 2000, true, 0);
    chassis.tank(30, 30);
    jamManager.enable_anti_jam(true);
    intake(600);
    pros::delay(100);
    intake_stop();
    intake_lift.extend();
    jamManager.set_velocities(-90, 0, -600);
    pros::delay(100);
    unsigned int time_outtake = pros::millis();
    while(pros::millis() - time_outtake < 2000)
    {

        
        if((std::abs(storageMotor.get_actual_velocity()) < 100) || (storageMotor.get_efficiency() > 7 && storageMotor.get_efficiency() < 20))
        {
            storageMotor.move_voltage(-12000);
        }
        else {
            storageMotor.move_voltage(-8000);
        }
        pros::delay(10);
    }
    chassis.tank(25, 25);
    pros::delay(150);
    chassis.brake();
    intake_stop();
    intake_lift.retract();
    chassis.moveToPoint(-20.5, -20.5, 1000, {.forwards = false, .minSpeed = 45, .earlyExitRange = 5});
    
    intake_stop();
    chassis.turnToPoint(-21.3, 5.7, 2000, { .minSpeed = 45, .earlyExitRange = 50}, false);
    
    distanceReset(true, false, false, false, true);
    ltv.followPath(skills_1, {.q_x = 4, .q_y = 550, .q_theta = 160, .r_ang = 0.25, .r_vel = 1.35});
    
    score_longgoal();
    ltv.waitUntil(20);
    intake_stop();
    intake();
    ltv.waitUntil(45);
    matchload_state(true);
    ltv.waitUntilDone();
    chassis.tank(50, 50);
    pros::delay(1000);
    storageMotor.brake();
    intakeMotor.move_velocity(200);
    pros::delay(1000);
    chassis.brake();
    distanceReset(true, false);
    matchload_state(true);
    intake();

    rcl.start();
    chassis.moveToPoint(-50, chassis.getPose().y, 1500, {.forwards = false, .minSpeed = 70, .earlyExitRange = 8});
    
    chassis.moveToPose(-24,59, 270, 1500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 8.5});
    
    matchload_state(false);
    intake_stop();
    chassis.moveToPoint(29, 58, 1500, {.forwards = false, .minSpeed = 70});
    chassis.moveToPose(45, 42, 320, 1500, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(90, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    
    distanceReset(true, false, false, false, true);
    chassis.moveToPose(24, 47.5, 90, 1500, {.forwards = false, .minSpeed = 20}, false);
    
    chassis.tank(-0.65, 0, config, 300);
    rcl.end();
    jamManager.enable_anti_jam(true);
    
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    
    distanceReset(true, false);
    matchload_state(true);
    intake();

    matchload_state(true);
    intake();
    rcl.start();
    chassis.turnToPoint(56.5, 46.5, 1200, {.minSpeed = 30, .earlyExitRange = 5});
    
    score_midgoal();
    chassis.moveToPoint(56.5, 46.5,  2000, {.minSpeed = 20});
    
    chassis.waitUntil(7);
    intake_stop();
    intake();
    chassis.waitUntilDone();
    int col_time = 0;
    while(!chassis.detect_collision() && col_time < 600)
        
        chassis.tank(30, 30);
        col_time += 10;
        pros::delay(10);
    chassis.tank(30, 30);
    pros::delay(matchload_delay);
    chassis.brake();
    chassis.turnToPoint(26.5, 48, 1500, {.forwards = false, .minSpeed = 25, .earlyExitRange = 15});
    
    chassis.moveToPose(26.5, 48, 90, 1200, {.forwards = false, .minSpeed = 20});
    
    intake_stop();
    chassis.waitUntil(7);
    chassis.waitUntilDone();
    chassis.tank(-0.5, 0, config, 150);
    jamManager.enable_anti_jam(true);
    
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    
    distanceReset(true);



    //MIDGOAL
    intake();
    matchload_state(false);
    ltv.followPath(parkingzone_curve, {.q_x = 4.5, .q_y = 550, .q_theta = 65});
    
    score_longgoal();
    ltv.waitUntilDone();
    intake_stop();
    intake();
    rcl.end();

    std::vector<std::pair<float, float>> velocityData = {
    {0.00f, 0.137449f}, {0.02f, 0.122294f}, {0.04f, 0.117480f}, {0.06f, 0.147628f},
    {0.08f, 0.144711f}, {0.10f, 0.139913f}, {0.12f, 0.162864f}, {0.14f, 0.190873f},
    {0.16f, 0.203905f}, {0.18f, 0.219595f}, {0.20f, 0.240017f}, {0.22f, 0.265108f},
    {0.24f, 0.287801f}, {0.26f, 0.312827f}, {0.28f, 0.332407f}, {0.30f, 0.355553f},
    {0.32f, 0.383497f}, {0.34f, 0.415525f}, {0.36f, 0.441199f}, {0.38f, 0.448785f},
    {0.40f, 0.484185f}, {0.42f, 0.522956f}, {0.44f, 0.500458f}, {0.46f, 0.539618f},
    {0.48f, 0.584095f}, {0.50f, 0.596803f}, {0.52f, 0.592848f}, {0.54f, 0.652431f},
    {0.56f, 0.661443f}, {0.58f, 0.679596f}, {0.60f, 0.736456f}, {0.62f, 0.815490f},
    {0.64f, 0.898349f}, {0.66f, 0.914428f}, {0.68f, 0.945937f}, {0.70f, 1.014728f},
    {0.72f, 1.100764f}, {0.74f, 0.987562f}, {0.76f, 0.922663f}, {0.78f, 0.891802f},
    {0.80f, 0.892580f}, {0.82f, 0.869044f}, {0.84f, 0.939259f}, {0.86f, 0.998388f},
    {0.88f, 1.075087f}, {0.90f, 1.002472f}, {0.92f, 0.851732f}
    };
    
    intake();
    distanceReset(true);
    for (const auto& [time, velocity] : velocityData) {
        
        auto vel = velocity;
        if(vel < 0.5)
        {
            vel = 0.65;
        }
        if(time <= 0.2)
        {
            vel = velocity * 1.65;
        }
        if(time >= 0.2 && time <= 0.5)
        {
            vel = velocity * 2.2;
        }
        chassis.tank(vel, 0.2, config, 20);
        if(time >= 0.92)
        {
            break;
        }
    }
    chassis.tank(1, 0.5, config, 700);
    chassis.tank(0.75, 0.3, config, 450);
    jamManager.set_velocities(200, 0, 0);
    chassis.tank(0.8, 0.2, config, 600);
    
    matchload_state(true);
    u_int32_t startTime = pros::millis();
    while (
    distanceReset(false, false).y < -37 &&
    frontDistance.get() > 150 &&
    !chassis.detect_collision() &&
    pros::millis() - startTime > 600) {
        chassis.tank(0.6, 0.2, config, 10);
    }
    chassis.brake();
    chassis.setPose(62.5, -33, chassis.getPose().theta);
    distanceReset(true, false);
    chassis.turnToHeading(215, 2000, {}, false);
    
    distanceReset(true, false);
    rcl.start();
    chassis.moveToPoint(45, -48, 2000, {});

    chassis.turnToHeading(90, 2000);
    distanceReset(false, true, false, false, true);
    chassis.moveToPose(26, -48, 270, 1500, {.forwards = false, .minSpeed = 25}, false);
    
    score_longgoal_auton(600, allianceColor, 1000);
    intake();
    matchload_state(true);

    chassis.moveToPose(56.5, -47, 90, 2000, {.minSpeed = 30}, false);
    
    chassis.tank(0.45, 0, config, matchload_delay);
    distanceReset(true);

    rcl.start();
    chassis.moveToPoint(50, chassis.getPose().y, 1500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 8});
    
    intake_stop();
    intakeMotor.move_velocity(200);
    chassis.moveToPose(24,-61, 90, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 7});
    
    intake_stop();
    matchload_state(false);
    chassis.moveToPoint(-29, -58.5, 1500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 8});
    
    rcl.end();
    chassis.moveToPose(-45, -45, 130, 1500, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(270, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    
    distanceReset(true);
    chassis.moveToPoint(-25.5, -46, 1300, {.forwards = false, .minSpeed = 40}, false);
    
    chassis.tank(-0.5, 0, config, 150);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    distanceReset(true, false);

    //PARKING ZONE
    matchload_state(true);
    intake();
    distanceReset(true, false);
    rcl.start();
    chassis.turnToPoint(-56.5, -46.5, 1200, { .minSpeed = 30, .earlyExitRange = 5});
    
    chassis.moveToPose(-56.5, -46.5, 90, 1200, {.minSpeed = 30}, false);
    
    int collision_timeout = 0;
    while(chassis.isInMotion())
    {
        if(chassis.detect_collision())
        {
            collision_timeout += 10;
        }
        if(collision_timeout > 100)
        {
            chassis.cancelMotion();
        }
        pros::delay(10);
    }
    chassis.tank(0.3, 0, config, matchload_delay);
    
    chassis.brake();
    distanceReset(true);
    chassis.turnToPoint(-25.5, -47.5, 1200, {.forwards = false, .minSpeed = 35, .earlyExitRange = 25});
    
    intake_stop();
    intakeMotor.move_velocity(200);
    chassis.moveToPose(-25.5, -47.5, 270, 1550, {.forwards = false, .minSpeed = 20}, false);
    
    intake_stop();
    jamManager.enable_anti_jam(true);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    rcl.end();
    matchload_state(false);
    distanceReset(true);
    
    ltv.followPath(parkingzone_curve2, {.q_x = 4.5, .q_y = 550, .q_theta = 60});
    
    ltv.waitUntil(4.5);
    score_midgoal();
    ltv.waitUntil(9);
    intake_stop();
    intake();
    ltv.waitUntilDone();
    intake();
    chassis.tank(1.5, -1.2, config, 520);
    chassis.brake();
}
