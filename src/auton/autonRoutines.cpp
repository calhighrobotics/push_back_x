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
#include <map>
#include <sys/types.h>

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
    chassis.tank(-1.3, 0, config, 150);
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
    chassis.tank(0, 0, config, 1000);
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
    chassis.tank(0.65, 0, config, 550);
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
    chassis.moveToPoint(-26.5, -21.8, 2000,  {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(10);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-27.5, -38, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(-27.5, -38, 25, 1000,  {.forwards = false, .minSpeed = 70, .earlyExitRange = 8});
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1500, {.minSpeed = 127, .earlyExitRange = 30});
    chassis.turnToHeading(265, 1000, {.minSpeed = 127}, false);
    score_longgoal_auton(600, allianceColor, 1000);
    distanceReset(true);
    matchload_state(false);
    chassis.moveToPose(-46, -59.5, 260, 1000, {.minSpeed = 60, .earlyExitRange = 7});
    chassis.moveToPoseRamsete(-4.5, -58.5, 270, 2000, config, {.forwards = false});
    chassis.waitUntil(4);
    descore.retract();
}   

void right_7_wing()
{
    chassis.setPose(-47, -16.5, 90);
    ltv.followPath(right_7_1, {.q_x = 6.5, .q_y = 100, .q_theta = 100, .r_ang = 0.25, .r_vel = 1});
    intake();
    ltv.waitUntil(18);
    matchload_state(true);
    ltv.waitUntilDone();
    chassis.tank(0.55, 0, config, 450);
    distanceReset(true);
    chassis.moveToPoint(-26, -47.5, 2000, {.forwards = false, .minSpeed = 60}, false);
    score_longgoal_auton(600, allianceColor,1500);
    distanceReset(true);
    matchload_state(false);
    chassis.moveToPose(-34, -60, 240, 1000, {.minSpeed = 60, .earlyExitRange = 9});
    chassis.turnToPoint(-3, -58, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 30});
    distanceReset(true, true, 3.5);
    chassis.moveToPoint(-1.2, -58, 2000, {.forwards = false, .minSpeed = 40});
    chassis.waitUntil(4);
    descore.retract();
    chassis.waitUntilDone();
    chassis.tank(0,0, config, 1000);
}

void right_7_hood()
{
    chassis.setPose(-47, -16.5, 90);
    ltv.followPath(right_7_1, {.q_x = 6.5, .q_y = 100, .q_theta = 70, .r_ang = 0.25, .r_vel = 1});
    intake();
    ltv.waitUntil(18);
    matchload_state(true);
    ltv.waitUntilDone();
    chassis.tank(0.55, 0, config, 450);
    distanceReset(true);
    chassis.moveToPoint(-26, -48, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 5}, false);
    score_longgoal_auton(600, allianceColor,1500);
    distanceReset(true);
    matchload_state(false);
    intake_stop();
    chassis.tank(0.65, 0, config, 250);
    hood.retract();
    chassis.tank(-0.82, 0, config, 200);
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
    chassis.turnToPoint(-26.2, 37.5, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(-26.2, 37.5, 150, 1000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 8});
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1500, {.minSpeed = 127, .earlyExitRange = 30});
    chassis.turnToHeading(270, 1000, {.minSpeed = 127}, false);
    chassis.tank(-1.5, 0, config, 150);
    score_longgoal_auton(12000, allianceColor, 1000);
    distanceReset(false, true, false, false, true);
    distanceReset(true, true, 5.5);
    chassis.turnToPoint(-54, 46, 2000, {.minSpeed = 35, .earlyExitRange = 30}, false);
    chassis.moveToPoint(-54, 46, 2000, {.minSpeed = 20});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntilDone();
    chassis.tank(0.85, 0, config, matchload_delay + 300);
    distanceReset(true, false);
    chassis.tank(-2.0, 0, config, 150);
    chassis.moveToPose(-7,9, 314.5, 1500,  {.forwards = false, .lead = 0.7, .minSpeed = 30}, false);
    chassis.tank(-0.6, 0, config, 250);
    score_midgoal_auton(600, allianceColor, 600);
    chassis.tank(0.85, 0, config, 250);
    mid_descore.extend();
    chassis.tank(-1.2, 0, config, 450);
    intake_stop();
    chassis.moveToPoint(-36, 40, 2000, {});
    matchload_state(false);
    chassis.turnToPoint(-9, 41, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 6});
    distanceReset(true, true, 5);
    chassis.moveToPose(-9, 41, 270,2000, {.forwards = false, .lead = 0.5});
    chassis.waitUntil(6);
    descore.retract();

}


void awp_auton() {
    jamManager.enable_anti_jam(false);
    chassis.setPose(-49.7, -14, 180);
    chassis.moveToPoint(-49.7, -45, 1500, {});
    chassis.waitUntil(5);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-69, -47, 1500, {}, false);
    distanceReset(true);
    intake();
    chassis.moveToPoint(-56.5, -47, 1000, {.minSpeed = 50}, false);
    chassis.waitUntilDone();
    chassis.tank(0.7, 0, config, 400);
    distanceReset(true);
    chassis.moveToPose(-28, -48, 270, 1500, {.forwards = false, .minSpeed = 60, .earlyExitRange = 5}, false);
    score_longgoal_auton(600, allianceColor,1000);
    score_longgoal(600);
    matchload_state(false);
    chassis.turnToPoint(-24, -22, 900, {.minSpeed = 127}, false);
    intake_stop();
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
    chassis.turnToPoint(-27.5, 48, 1200, {.forwards = false, .minSpeed = 60, .earlyExitRange = 30});
    distanceReset(true, true, 5.5);
    chassis.moveToPoint(-27.5, 48, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 7}, false);
    matchload_state(true);
    score_longgoal_auton(600, allianceColor, 1400);
    distanceReset(true);
    chassis.moveToPose(-55.5, 46.7, 270, 2000, {.lead = 0.3, .minSpeed = 40});
    intake();
    chassis.waitUntilDone();
    chassis.tank(0.55, 0, config, 700);
    distanceReset(true, true,3.5);
    chassis.tank(-2.0, 0, config, 150);
    chassis.moveToPoseRamsete(-12.7, 9.5, 320, 1500, config, {.forwards = false, .minSpeed = 0.7, .lead = 0.7}, false);
    score_midgoal_auton(600, allianceColor, 5000);
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
    pros::delay(700);
    chassis.tank(1.5, 0, config, 200);
    chassis.brake();
    pros::delay(900);
    chassis.tank(-0.2, 0, config, 300);
    chassis.brake();
    pros::delay(800);
    chassis.tank(-1.2, 0, config, 520);

    chassis.brake();
    chassis.turnToHeading(270, 2000, {}, false);
    chassis.setPose(-44.6, 0, chassis.getPose().theta);
    //chassis.setPose(-44.6, 0, 270);
    distanceReset(true, false);
    intake();
    chassis.moveToPoint(-40, 0, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 7});
    jamManager.enable_anti_jam(true);
    intake_stop();
    chassis.turnToPoint(-27, -21, 1000, {.minSpeed = 40, .earlyExitRange = 10});
    intake();
    chassis.moveToPoint(-27, -21, 1500, {.minSpeed = 20, .earlyExitRange = 5}, false);
    chassis.turnToPoint(-12.2, -14.6, 1000, {}, false);
    chassis.moveToPose(-12.2, -14.6,47, 1500, {.lead = 0.35}, false);
    chassis.tank(0.15, 0, config, 250);
    chassis.tank(10, 10);
    intake_lift.extend();
    jamManager.set_velocities(-600, 0, -600);
    pros::delay(550);
    jamManager.set_velocities(-170, 0, -250);
    pros::delay(600);
    jamManager.set_velocities(-60, 600, -100);
    pros::delay(2200);
    intake_lift.retract();
    chassis.moveToPoint(-20.5, -20.5, 1000, {.forwards = false, .minSpeed = 45, .earlyExitRange = 5});
    intake_stop();
    chassis.turnToPoint(-21.3, 5.7, 2000, { .minSpeed = 45, .earlyExitRange = 50}, false);
    distanceReset(true, false, false, false, true);
    ltv.followPath(skills_1, {.q_x = 4, .q_y = 500, .q_theta = 160, .r_ang = 0.25, .r_vel = 1.35});
    ltv.waitUntil(5);
    intake();
    ltv.waitUntil(30);
    matchload_state(true);
    ltv.waitUntilDone();
    chassis.tank(50, 50);
    pros::delay(matchload_delay);
    chassis.brake();
    distanceReset(true, false);
    chassis.moveToPoint(-50, chassis.getPose().y, 1500, {.forwards = false, .minSpeed = 70, .earlyExitRange = 8});
    chassis.moveToPose(-24,59, 270, 1500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 8.5});
    matchload_state(false);
    chassis.waitUntilDone();
    distanceReset(true);
    chassis.moveToPoint(29, 58, 1500, {.forwards = false, .minSpeed = 70});
    chassis.waitUntil(5);
    distanceReset(false, true, false, false, true);
    chassis.waitUntil(15);
    distanceReset(false, true, false, false, true);
    chassis.waitUntilDone();
    distanceReset(true);
    chassis.moveToPose(45, 48, 130, 1500, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(90, 1500, {}, false);
    distanceReset(true);
    chassis.moveToPoint(27.5, 48, 1500, {.forwards = false, .minSpeed = 20}, false);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    distanceReset(true, false);
    matchload_state(true);
    intake();
    chassis.moveToPoint(55.5, 47, 1200, {.minSpeed = 10}, false);
    chassis.tank(0.4, 0, config, matchload_delay);
    chassis.brake();
    distanceReset(true);
    chassis.turnToPoint(27.5, 48, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 30});
    chassis.moveToPose(27.5, 48, 90, 1200, {.forwards = false, .minSpeed = 20});
    chassis.waitUntil(7);
    distanceReset(true);
    chassis.waitUntilDone();
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    distanceReset(true);
    matchload_state(false);
    //MIDGOAL
    intake();
    ltv.followPath(parkingzone_curve, {.q_x = 4.5, .q_y = 550, .q_theta = 60});
    ltv.waitUntilDone();

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
        chassis.tank(velocity, 0, config, 20);
        if(time >= 0.92)
        {
            break;
        }
    }
    chassis.tank(1.3, 0.5, config, 600);
    chassis.tank(0.75, 0.5, config, 650);
    matchload_state(true);
    while(distanceReset(false, false).y < -32 && frontDistance.get() > 150) {
        chassis.tank(0.6, 0.5, config, 10);
    }
    chassis.brake();
    chassis.setPose(62.5, -31, chassis.getPose().theta);
    distanceReset(true, false);
    chassis.turnToHeading(270, 2000, {}, false);
    distanceReset(true, false);
    chassis.turnToPoint(22, -20, 2000, {.minSpeed = 35, .earlyExitRange = 20});
    intake();
    chassis.moveToPoint(22, -20, 2000, {});
    chassis.waitUntil(4.5);
    matchload_state(false);
    distanceReset(true);
    chassis.waitUntilDone();
    matchload_state(true);
    distanceReset(true, true, 5);
    chassis.turnToPoint(7, -7, 1200, {.forwards = false, .minSpeed = 35, .earlyExitRange = 20});
    chassis.moveToPose(7, -7, 135, 1200, {.forwards = false, .minSpeed = 20}, false);
    u_int32_t startTime = pros::millis();
    while((get_color() == Color::RED || get_color() == Color::NONE) && pros::millis() - startTime < 2500) {
    {
        score_midgoal_auton(250, Color::NONE, 10);
    }
    intake_stop();

    //LONGGOAL #2
    
    chassis.moveToPoint(40, -47, 1500, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(10);
    intake();
    chassis.turnToHeading(90, 1200, {}, false);
    distanceReset(true, false);
    chassis.moveToPoint(54.5, -47, 1500, {.minSpeed = 20}, false);
    chassis.waitUntilDone();

    chassis.tank(0.42, 0, config, matchload_delay);
    chassis.brake();
    distanceReset(true);
    chassis.moveToPoint(50, chassis.getPose().y, 1500, {.forwards = false, .minSpeed = 60, .earlyExitRange = 7});
    chassis.moveToPose(24,-59, 270, 1500, {.forwards = false, .minSpeed = 60, .earlyExitRange = 7});
    matchload_state(false);
    chassis.waitUntilDone();
    distanceReset(true);
    chassis.moveToPoint(-29, -59, 1500, {.forwards = false, .minSpeed = 20}, false);
    chassis.waitUntilDone();
    distanceReset(true);
    chassis.moveToPose(-45, -48, 130, 1500, {.forwards = false, .minSpeed = 20});
    chassis.turnToPoint(-27.5, -48, 1500, {}, false);
    distanceReset(true);
    chassis.moveToPoint(-27.5, -48, 1500, {.forwards = false, .minSpeed = 20}, false);
    chassis.tank(-0.5, 0, config, 200);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    distanceReset(true, false);

    //PARKING ZONE
    matchload_state(true);
    intake();
    chassis.turnToHeading(270, 1200, {}, false);
    distanceReset(true, false);
    chassis.moveToPose(-54.5, -46.5, 270, 1200, {.minSpeed = 30}, false);
    chassis.tank(0.44, 0, config, matchload_delay);
    chassis.brake();
    distanceReset(true);
    chassis.turnToPoint(-27.5, -48, 1200, {.minSpeed = 35, .earlyExitRange = 25});
    chassis.moveToPoint(-27.5, -48, 1200, {.forwards = false, .minSpeed = 20}, false);
    score_longgoal_auton(600, Color::NONE, longgoal_delay);
    matchload_state(false);
    distanceReset(true);
    chassis.moveToPose(-63.5, -20, 0, 2500, {}, false);
    chassis.tank(0.7, 0, config, 500);
}
}