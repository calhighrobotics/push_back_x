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



void precompute_auton_paths() {
    std::vector<std::string> paths = {};
}

void test_auton()
{
    chassis.setPose(-54,46.5,270);
    distanceReset(true);
    ltv.followPath(skills_2, {.backwards = true, .q_x = 5, .q_y = 500, .q_theta = 100, .r_ang = 0.25, .r_vel = 1});
}

void left_rush()
{
    descore.extend();
    chassis.setPose(-51.6, 16.5, 90);
    chassis.moveToPoint(-26.5, 21.8, 2000, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(10);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-26.3, 36, 1500, {.forwards = false, .minSpeed = 45, .earlyExitRange = 30});
    chassis.moveToPose(-26.3, 36, 160, 1000, {.forwards = false, .minSpeed = 85, .earlyExitRange = 10});
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1500, {.minSpeed = 127, .earlyExitRange = 30});
    chassis.turnToHeading(270, 1000, {.minSpeed = 127}, false);
    chassis.tank(-127, -127);
    pros::delay(150);
    chassis.tank(0,0);
    score_longgoal_auton(12000, allianceColor, 1000);
    matchload_state(false);
    distanceReset(true, true, 2.5);
    chassis.moveToPose(-37, 39, 240, 1500, {.minSpeed = 60, .earlyExitRange = 9});
    chassis.turnToPoint(-2.5, 38, 1000, {.forwards = false, .minSpeed = 45, .earlyExitRange = 35});
    distanceReset(true);
    chassis.moveToPose(-2.5, 38, 270, 1500, {.forwards = false, .minSpeed = 20});
    chassis.waitUntil(5.5);
    descore.retract();
    chassis.waitUntilDone();
    chassis.tank(0,0);
}


void right_auton_split()
{
    const float matchload_delay = 380;

    chassis.setPose(-49.7, -14, 180);
    ltv.followPath(awp_1, {.q_x = 155, .q_y = 450, .q_theta = 140});
    intake();
    ltv.waitUntil(5);
    matchload_state(true);
    ltv.waitUntilDone();
    chassis.tank(60,60);
    pros::delay(550);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPose(-28, -48, 270, 1500, {.forwards = false, .minSpeed = 60, .earlyExitRange = 5}, false);
    score_longgoal_auton(600, allianceColor,1000);
    distanceReset(true);
    matchload_state(false);
    chassis.turnToPoint(-27.5, -22, 900, {}, false);
    intake();
    chassis.moveToPoint(-24, -22, 2000, {.minSpeed = 55, .earlyExitRange = 8});
    chassis.waitUntil(11);
    matchload_state(true);
    chassis.waitUntilDone();
    matchload_state(false);
    chassis.moveToPose(-16, -12.2, 43, 1300, {.lead = 0.3}, false);
    matchload_state(false);
    intake();
    chassis.waitUntil(7.5);
    matchload_state(true);
    chassis.waitUntil(12);
    matchload_state(false);
    chassis.waitUntilDone();
    outtake();
    pros::delay(1500);
    intake_stop();
    chassis.moveToPoint(-39, -35, 2000, {.forwards = false}, false);
    chassis.turnToPoint(-14, -37, 2000, {.minSpeed = 45, .earlyExitRange = 30}, false);
    distanceReset(true, true, 7);
    chassis.moveToPose(-11.5, -37, 90,2000, {.lead = 0.2, .minSpeed = 20});
    chassis.waitUntil(5.5);
    descore.retract();
    chassis.waitUntilDone();
    chassis.turnToHeading(60, 2000, {.minSpeed = 35}, false);

}

void right_rush()
{
    descore.extend();
    chassis.setPose(-51.6, -16.5, 90);
    chassis.moveToPoint(-26.5, -21.8, 2000, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(10);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-27.5, -38, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(-27.5, -38, 25, 1000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 8});
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1500, {.minSpeed = 127, .earlyExitRange = 30});
    chassis.turnToHeading(270, 1000, {.minSpeed = 127}, false);
    chassis.tank(-127, -127);
    pros::delay(150);
    chassis.tank(0,0);
    score_longgoal_auton(600, allianceColor, 1000);
    distanceReset(true);
    matchload_state(false);
    chassis.moveToPose(-43, -60, 240, 1000, {.minSpeed = 60, .earlyExitRange = 9});
    chassis.turnToPoint(-3, -58, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 30});
    chassis.moveToPose(-3, -58, 270, 2000, {.forwards = false, .minSpeed = 40});
    chassis.waitUntil(4);
    descore.retract();
}   

void right_7()
{
    chassis.setPose(-47, -16.5, 90);
    ltv.followPath(right_7_1, {.q_x = 6.5, .q_y = 100, .q_theta = 70, .r_ang = 0.25, .r_vel = 1});
    intake();
    ltv.waitUntil(18);
    matchload_state(true);
    ltv.waitUntilDone();
    chassis.tank(40,40);
    pros::delay(450);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPoint(-26, -48, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 5}, false);
    score_longgoal_auton(600, allianceColor,1500);
    distanceReset(true);
    matchload_state(false);
    chassis.moveToPose(-34, -60, 240, 1000, {.minSpeed = 60, .earlyExitRange = 9});
    chassis.turnToPoint(-3, -58, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 30});
    chassis.moveToPose(-3, -58, 270, 2000, {.forwards = false, .minSpeed = 40});
    chassis.waitUntil(4);
    descore.retract();
}

void carry_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 4, 10000);
}

void left_auton_split() {
    const float matchload_delay = 380;

    descore.extend();
    chassis.setPose(-51.6, 16.5, 90);
    chassis.moveToPoint(-26.5, 21.8, 2000, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(10);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-26, 36.3, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(-26, 36.3, 150, 1000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 8});
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1500, {.minSpeed = 127, .earlyExitRange = 30});
    chassis.turnToHeading(270, 1000, {.minSpeed = 127}, false);
    chassis.tank(-127, -127);
    pros::delay(150);
    chassis.tank(0,0);
    score_longgoal_auton(12000, allianceColor, 1000);
    distanceReset(true, true, 2.5);
    chassis.moveToPose(-55, 46.5, 270, 3000, {.lead = 0.2});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntilDone();
    chassis.tank(65, 65);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPoint(-47, 46.5, 1000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 7});
    chassis.moveToPose(-10.7, 10.7, 318, 2000, {.forwards = false}, false);
    score_midgoal_auton(600, allianceColor, 1000);
    intake_stop();
    chassis.moveToPoint(-36, 35, 2000, {});
    matchload_state(false);
    chassis.turnToPoint(-7.2, 37.5, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 6});
    distanceReset(true, true, 3.5);
    chassis.moveToPose(-7.2, 37.7, 270,2000, {.forwards = false, .lead = 0.5});
    chassis.waitUntil(6);
    descore.retract();
}

void elim_auton() {
    descore.extend();
    chassis.setPose(-44.77, -12.31,90);
    chassis.moveToPose(-24, -24, 145, 3000, {}); //24, 21
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(21);
    matchload_state(true);
    chassis.waitUntilDone();
    matchload_state(false);
    chassis.turnToPoint(-46, -47, 2000, {.forwards = false});
    chassis.moveToPoint(-46, -47, 3500, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(7);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-72, -46.5, 1200, {}, false);
    distancePose pose = distanceReset(true);
    intake();
    chassis.moveToPoint(-59.5, -46.5, 800, {}, false);
    chassis.tank(45, 45);
    pros::delay(350);
    chassis.tank(0,0);
    chassis.turnToPoint(-22, -47.5, 2000, {.forwards = false}, false);
    pose = distanceReset(true);
    if(std::abs(pose.y - 47) > 2.2)
    {
        chassis.setPose(chassis.getPose().x, -47, chassis.getPose().theta);
    }
    intake_stop();
    chassis.moveToPoint(-22 - longgoal_offset - 3.5, -47, 1600, {.forwards = false, .minSpeed = 15}, false);
    color_sort_enable = true;
    score_longgoal_auton(12000, allianceColor, 2700);
    intake_stop();
    color_sort_enable = false;
    matchload_state(false);
    intake_stop();
    distanceReset(true);
    descore.retract();
    chassis.moveToPoint(-48, -47.5, 1500, {}, false);
    chassis.moveToPose(-22 + longgoal_offset + 3, -47.5 + 12.3, 270, 2000, {.forwards = false , .lead = 0.4, .minSpeed = 30, .earlyExitRange = 8});
    chassis.moveToPose(-16, -47.5 + 12, 270, 3000, {.forwards = false, .lead = 0.3});
}

void awp_auton() {
    jamManager.enable_anti_jam(true);
    chassis.setPose(-49.7, -14, 180);
    ltv.followPath(awp_1, {.q_x = 135, .q_y = 450, .q_theta = 250});
    intake();
    ltv.waitUntil(5);
    matchload_state(true);
    ltv.waitUntilDone();
    chassis.tank(65,65);
    pros::delay(570);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPose(-27.5, -48, 270, 1500, {.forwards = false, .minSpeed = 60, .earlyExitRange = 5}, false);
    score_longgoal_auton(600, allianceColor,1200);
    matchload_state(false);
    chassis.turnToPoint(-24, -22, 900, {}, false);
    distanceReset(false, true, false, false, true);
    ltv.followPath(awp_2, {.q_x = 6.5, .q_y = 450, .q_theta = 190, .r_ang = 0.2, .r_vel = 1.35});
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
    distanceReset(false, false, true, false, true);
    chassis.turnToPoint(-27.5, 48, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 30});
    distanceReset(true, false);
    chassis.moveToPoint(-27.5, 48, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 7}, false);
    matchload_state(true);
    score_longgoal_auton(600, allianceColor, 1400);
    distanceReset(true, false);
    chassis.moveToPose(-54.5, 46.7, 270, 2000, {.lead = 0.1, .minSpeed = 40});
    intake();
    chassis.waitUntilDone();
    chassis.tank(60, 60);
    pros::delay(650);
    chassis.tank(0,0);
    distanceReset(true, false);
    chassis.moveToPoint(-50, 46.5, 1500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 8});
    chassis.moveToPose(-7.5, 9.5, 320, 2000, {.forwards = false, .minSpeed = 40}, false);
    chassis.tank(-40, -40);
    pros::delay(100);
    score_midgoal_auton(600, allianceColor, 5000);
    return;
    chassis.moveToPoint(-35, 37, 1500, {.minSpeed = 40, .earlyExitRange = 9});
    chassis.turnToPoint(-2.5, 36.5, 1000, {.forwards = false, .minSpeed = 45, .earlyExitRange = 35});
    distanceReset(true, true, 4.5);
    chassis.moveToPose(-2.5, 36.5, 270, 1500, {.forwards = false, .minSpeed = 20});
    chassis.waitUntil(5.5);
    descore.retract();
    chassis.waitUntilDone();
    chassis.tank(0,0);
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

    chassis.setPose(-44.6, 0, 270);
    distanceReset(true);
    intake();
    chassis.moveToPoint(-38, 0, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 7});
    chassis.turnToPoint(-28.5, -20, 1000, {.minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-28.5, -20, 1500, {.minSpeed = 35, .earlyExitRange = 7});
    chassis.turnToPoint(-11, -11, 1000, {.minSpeed = 35, .earlyExitRange = 20});
    chassis.moveToPose(-11, -10.7,45, 1500, {}, false);
    outtake(450);
    pros::delay(lowgoal_delay);
    intake_lift.retract();
    chassis.moveToPoint(-21.3, -20, 1000, {.forwards = false, .minSpeed = 45, .earlyExitRange = 7});
    intake_stop();
    chassis.turnToPoint(-21.3, 5.7, 2000, { .minSpeed = 45, .earlyExitRange = 50}, false);
    ltv.followPath(skills_1, {.q_x = 6.5, .q_y = 500, .q_theta = 190});
    ltv.waitUntil(5);
    intake();
    ltv.waitUntil(32);
    matchload_state(true);
    ltv.waitUntil(41);
    ltv.waitUntilDone();
    chassis.tank(90,90);
    pros::delay(400);
    chassis.tank(20, 20);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    pros::delay(500);
    distanceReset(true);
    ltv.followPath(skills_2, {.backwards = true, .q_x_b = 4, .q_y_b = 450, .q_theta_b = 200, .r_ang_b = 0.2, .r_vel_b = 1});
    ltv.waitUntil(3);
    intake_stop();
    ltv.waitUntil(7);
    matchload_state(false);
    ltv.waitUntilDone();
    
    chassis.tank(-90, -90);
    pros::delay(220);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    distanceReset(true, false);
    matchload_state(true);
    intake();
    chassis.moveToPose(54, 46.5, 90, 1200, {.lead = 0.3, .minSpeed = 10}, false);
    chassis.tank(30, 30);

    pros::delay(matchload_delay);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPoint(27, 48, 1200, {.forwards = false, .minSpeed = 20, .earlyExitRange = 4}, false);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    matchload_state(false);
    distanceReset(true);
    //MIDGOAL
    chassis.turnToPoint(34, 0, 2000, {.minSpeed = 127, .earlyExitRange = 10});
    chassis.moveToPose(34, 0, 180, 1300, {}, false);
    distanceReset(true);
    chassis.turnToPoint(69, 0, 1000, {});
    distanceReset(true);
    chassis.moveToPoint(45, 0, 2000, {}, false);
    chassis.moveToPoint(38, 0, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 7});
    distanceReset(true);
    chassis.turnToPoint(22, -20, 2000, {.minSpeed = 35, .earlyExitRange = 20});
    intake();
    chassis.moveToPoint(22, -20, 2000, {}, false);
    matchload_state(true);
    chassis.turnToPoint(8, -8, 1200, {.forwards = false, .minSpeed = 35, .earlyExitRange = 20});
    chassis.moveToPose(8, -8, 135, 1200, {.forwards = false, .minSpeed = 20}, false);
    score_midgoal_auton(300, Color::NONE, midgoal_delay);

    //LONGGOAL #2
    
    chassis.moveToPoint(40, -44.5, 1500, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.turnToPoint(54, -44.5, 1500, {}, false);
    distanceReset(true);
    chassis.moveToPoint(54, -46.5, 1500, {.minSpeed = 20}, false);
    chassis.waitUntil(10);
    intake();
    chassis.waitUntilDone();

    chassis.tank(35, 35);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    distanceReset(true);
    ltv.followPath(skills_4, {.backwards = true, .q_x = 7.5, .q_y = 450, .q_theta = 200});
    ltv.waitUntil(3);
    intake_stop();
    ltv.waitUntil(7);
    matchload_state(false);
    ltv.waitUntilDone();
    chassis.tank(-80, -80);
    pros::delay(220);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    distanceReset(true, false);

    //PARKING ZONE
    matchload_state(true);
    intake();
    chassis.moveToPose(-54, -46.5, 270, 1200, {.minSpeed = 30}, false);
    chassis.tank(35, 35);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPoint(-25, -48, 1200, {.forwards = false, .minSpeed = 20, .earlyExitRange = 4}, false);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    matchload_state(false);
    distanceReset(true);
    chassis.turnToPoint(-34, 0, 2000, {.minSpeed = 127, .earlyExitRange = 10});
    chassis.moveToPose(-34, 0, 0, 1300, {}, false);
    chassis.turnToPoint(-69, 0, 1000, {}, false);
    distanceReset(true);
}


