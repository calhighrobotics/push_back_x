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
    //velocity_test(config, 1.3, 2000, 500);
    //ltv.followPath(test_straight, {.backwards = false, .log = true, .test = true});
    chassis.setPose(0, 29.52, 90);
    distanceReset(true);
    ltv.followPath(test_straight, {.backwards = false, .log = true});
    //ltv.followPath(test_u, {.forwards = true, .log = true, .test = true});
}

void right_auton_split()
{
    const float matchload_delay = 380;

    descore.extend();
    chassis.setPose(-51.6, -16.5, 90);
    chassis.moveToPoint(-26.5, -21.8, 2000, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntil(10);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-27.7, -42, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(-27.7, -42, 7, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 7});
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1500, {.minSpeed = 127}, false);
    score_longgoal_auton(12000, allianceColor, 1500);
    distancePose pose = distanceReset(true, false, false, false, true);
    std::cout << pose.x << "," << pose.y << std::endl;
    chassis.moveToPose(-53.5, -46.5, 270, 1200, {.lead = 0.2});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntilDone();
    chassis.tank(20, 20);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPoint(-29, -24, 2000, {.forwards = false, .minSpeed = 35});
    chassis.turnToPoint(0,0, 2000, {.minSpeed = 30, .earlyExitRange = 5}, false);
    intake();
    matchload_state(false);
    chassis.moveToPose(-16.7, -12, 45, 2000, {.lead = 0.2}, false);
    outtake();
    pros::delay(1500);
    intake_stop();
    chassis.moveToPoint(-39, -35, 2000, {.forwards = false}, false);
    chassis.turnToPoint(-13, -35, 2000, {.minSpeed = 30, .earlyExitRange = 6}, false);
    chassis.moveToPose(-13, -35.2, 90,2000, {.lead = 0.1});
}

void right_rush()
{

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
    chassis.turnToPoint(-28.3, 42, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(-28.3, 42, 150, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 8});
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1500, {.minSpeed = 127}, false);
    score_longgoal_auton(12000, allianceColor, 1500);
    distancePose pose = distanceReset(true);
    std::cout << pose.x << "," << pose.y << std::endl;
    chassis.moveToPose(-53.5, 45.5, 270, 1200, {.lead = 0.2});
    chassis.waitUntil(3);
    intake();
    chassis.waitUntilDone();
    chassis.tank(20, 20);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPoint(-11, 11, 2000, {.forwards = false}, false);
    score_midgoal_auton(600, allianceColor, 1000);
    intake_stop();
    chassis.moveToPoint(-39, 35, 2000, {});
    matchload_state(false);
    chassis.turnToPoint(-13, 36.5, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 6});
    chassis.moveToPose(-13, 36.5, 270,2000, {.forwards = false, .lead = 0.1});
}

void elim_auton() {
    descore.extend();
    chassis.setPose(-44.77, -12.31, 90);
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
    chassis.setPose(-24, -15, 180);


}

void skills_auton() {
    descore.extend();
    color_sort_enable = false;
    int start;
    distancePose pose;

    const int longgoal_delay = 2250;
    const int midgoal_delay = 1500;
    const int matchload_delay = 1350;
    const int mid_triball_delay = 500;
    const int longgoal_offset = 7;
    const int midgoal_offset = 11.7;
    const int matchload_offset = 11.4 + 0.7;
    const int triball_delay = 500;
    const int dual_ball_delay = 500;

    chassis.setPose(-53, 0, 270);
    intake();
    pros::delay(250);
    chassis.tank(90,90);
    pros::delay(500);
    chassis.tank(50,50);
    pros::delay(700);
    chassis.tank(0,0);
    pros::delay(200);
    chassis.tank(-70,-70);
    pros::delay(550);
    chassis.tank(90,90);
    pros::delay(500);
    chassis.tank(0,0);
    pros::delay(600);
    chassis.turnToHeading(270, 300, {}, false);
    int base_sum = 0;
    for(int i = 0; i < 5; i++)
    {
        base_sum += imu.get_roll();
        pros::delay(10);
    }
    int base = base_sum / 5;
    intake_stop();
    chassis.tank(-127, -127);
    intake();
    start = pros::millis();
    while(imu.get_roll() < base + 5.5 && (pros::millis() - start < 1000))
    {
        pros::delay(10);
    }
    start = pros::millis();
    while(imu.get_roll() > base && (pros::millis() - start < 750))
    {
        pros::delay(10);
    }
    intake_stop();
    chassis.tank(0,0);
    chassis.tank(50,50);
    pros::delay(700);
    chassis.tank(0,0);
    chassis.turnToHeading(270, 1000);
    chassis.setPose(-45.7, 0, chassis.getPose().theta);
    pose = distanceReset(true, false);
    pros::delay(20);
    chassis.moveToPoint(-45.7 + 5, 0, 2000, {.forwards = false}, false);
    chassis.turnToPoint(0,0, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    pros::delay(20);
    distanceReset(true, false);
    chassis.moveToPose(-22.4, 22.4, 320, 1800, {.lead = 0.5});
    chassis.waitUntil(15);
    intake();
    chassis.waitUntilDone();
    chassis.moveToPose(-14, 14.5, 320, 700, {.forwards = false, .lead = 0.2});
    chassis.waitUntil(2);
    intake_stop();
    chassis.waitUntilDone();
    std::cout << "Pose before midgoal: " << chassis.getPose().x << ", " << chassis.getPose().y << ", " << chassis.getPose().theta << std::endl;

    midgoal_first = true;
    color_sort_enable = false;
    score_midgoal_auton();
    matchload_state(true);
    pros::delay(midgoal_delay);
    matchload_state(true);
    intake_stop();
    pros::delay(500);

    chassis.moveToPoint(-45, 47.5, 2000, {});
    chassis.waitUntil(3);
    color_sort_enable = false;
    chassis.waitUntilDone();
    chassis.turnToPoint(-72, 45.5, 2000, {}, false);
    pros::delay(20);
    distanceReset(true);
    pros::delay(20);
    chassis.moveToPoint(-59.2, 45.5, 2000, {}, false);
    intake();
    chassis.tank(60,60);
    pros::delay(matchload_delay + 500);
    chassis.tank(0,0);
    pros::delay(20);
    distanceReset(true);
    
    chassis.moveToPose(-22 + longgoal_offset, 61.5, 270, 1500, {.forwards = false, .lead = 0.3, .minSpeed = 60, .earlyExitRange = 7});
    chassis.waitUntil(3);
    intake_stop();
    chassis.moveToPose(37, 61.5, 270, 1700, {.forwards = false, .lead = 0, .maxSpeed = 100});
    chassis.waitUntil(3);
    matchload_state(false);
    chassis.waitUntilDone();
    pros::delay(20);
    distanceReset(true);
    chassis.swingToPoint(22, 48.5, lemlib::DriveSide::LEFT, 2500, {.forwards = false});
    chassis.moveToPoint(22 + longgoal_offset + 2, 50, 2000, {.forwards = false}, false);

    //alignToGoal(90);
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);
    matchload_state(true);
    distanceReset(true);
    pros::delay(20);
    intake_stop();
    matchload_state(true);
    intake();
    chassis.moveToPoint(59.2, 46.7,  2000, {.forwards = true}, false);
    chassis.tank(60,60);
    pros::delay(matchload_delay + 200);
    chassis.tank(0,0);
    chassis.turnToPoint(22 + longgoal_offset, 47.5, 1000, {.forwards = false}, false);
    chassis.moveToPoint(22 + longgoal_offset+2, 47.5, 2000, {.forwards = false, .minSpeed = 25});
    chassis.waitUntil(3);
    intake_stop();
    chassis.waitUntilDone();
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 350);
    pros::delay(20);
    distanceReset(true);
    

    relativeMotion(chassis.getPose().x, chassis.getPose().y, 90, 6, 200, true);
    distanceReset(true, false, false, false, true);
    chassis.turnToPoint(45, -46.5, 2000, {}, false);
    distanceReset(false, false, false, true, true);
    chassis.moveToPoint(45, -46.5, 6000, {.maxSpeed = 100}, false);
    matchload_state(true);
    
    distanceReset(false, false, true, false, true);
    chassis.turnToPoint(72, -46, 3000, {}, false);
    distanceReset(true);

    intake();
    chassis.moveToPoint(59.2, -47, 1000, {.minSpeed = 40}, false);
    chassis.tank(60, 60);
    pros::delay(matchload_delay + 600);
    chassis.tank(0,0);
    pros::delay(20);
    distanceReset(true);

    intake_stop();
    chassis.moveToPose(22 + longgoal_offset, -61.5, 90, 1500, {.forwards = false, .lead = 0.3, .minSpeed = 60, .earlyExitRange = 7});
    chassis.moveToPose(-37, -61.5, 90,  2000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(3);
    matchload_state(false);
    chassis.waitUntilDone();
    pros::delay(20);
    distanceReset(true);
    chassis.swingToPoint(-22, -48.5, lemlib::DriveSide::LEFT, 2000, {.forwards = false}, false);
    chassis.moveToPoint(-22 - longgoal_offset-2, -50, 1000, {.forwards = false, .minSpeed = 20}, false);
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);
    matchload_state(true);
    pros::delay(20);
    distanceReset(true);
    pros::delay(20);
    intake();
    chassis.moveToPoint(-59.2, -46.5, 2000, {}, false);
    chassis.tank(55,55);
    pros::delay(matchload_delay + 300);
    chassis.tank(0,0);
    chassis.turnToPoint(-22 - longgoal_offset, -47.5, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset - 2, -47.5, 2000, {.forwards = false}, false);
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);  
    pros::delay(20);
    distanceReset(true);
    pros::delay(20);
    matchload_state(false);

    chassis.moveToPose(-62.5, -15.7, 0, 3000, {.lead = 0.3}, false);
    intake();
    crossBarrier(1,false, true);
    chassis.tank(40,40);
    pros::delay(700);

    leftMotors.brake();
    rightMotors.brake();
}


