#include "globals.h" 
#include "auton/autonFunctions.h"
#include "lemlib/chassis/chassis.hpp"
#include "ltv.h"
#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include "velocityController.h"
#include "paths.h"
#include "ramsete.h"
#include "distanceReset.h"
#include "colorSort.h"
#include "crossBarrierDetection.h"
#include <type_traits>

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
5.8432642308,
0.213526937516,
1.14429410811,
0.906356177095,
0.347072436421,
11.4953431776,
54.5797495382,
};

RamsetePathFollower ramsete(config, 2.5, 0.7);
LTVPathFollower ltv(config);

void precompute_auton_paths() {
    std::vector<std::string> paths = {};
}

void right_auton()
{
    descore.extend();
    chassis.setPose(-44.77, -12.31, 90);
    trapDoor.extend();
    intake();
    chassis.moveToPose(-24, -24, 145, 3000, {}); //24, 21
    chassis.waitUntil(3);
    trapDoor.retract();
    chassis.waitUntil(21);
    matchload_state(true);
    chassis.waitUntilDone();
    matchload_state(false);
    chassis.turnToPoint(0,0, 3000,{},false);
    intake_stop();
    chassis.moveToPose(-11.7, -11.7, 45, 3000, {}, false);
    outtake(7000);
    pros::delay(2000);
    chassis.tank(0,0);
    intake_stop();
    chassis.moveToPoint(-46, -47, 3500, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(7);
    matchload_state(true);
    matchload.extend();
    chassis.waitUntilDone();
    chassis.turnToPoint(-72, -46.5, 3500, {}, false);
    distancePose pose = distanceReset(true, false, false, false, true);
    intake();
    chassis.moveToPoint(-56.5, -46.5, 700, {.minSpeed = 40}, false);
    chassis.tank(60, 60);
    pros::delay(120);
    chassis.tank(0,0);
    distanceReset(true);
    intake_stop();
    chassis.moveToPoint(-22 - longgoal_offset - 3.5, -46.7, 3000, {.forwards = false, .minSpeed = 15, .earlyExitRange = 1.5}, false);
    color_sort_enable = true;
    score_longgoal_auton(12000, allianceColor, 1700);
    intake_stop();
    color_sort_enable = false;
    matchload_state(false);
    intake_stop();
    distanceReset(true, false, false, false, true);
    descore.retract();
    chassis.moveToPoint(-48, -46.5, 3000, {}, false);
    chassis.moveToPose(-22 + longgoal_offset+1, -47.5 +13.7, 270, 5000, {.forwards = false , .lead = 0.3, .minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPose(-16, -47.5 + 12.5, 270, 3000, {.forwards = false, .lead = 0.3});
}

void carry_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 4, 10000);
}

void left_auton() {
    descore.extend();
    chassis.setPose(-44.77, 12.31, 90);
    trapDoor.extend();
    intake();
    chassis.moveToPose(-23, 23, 30, 3000, {});
    chassis.waitUntil(3);
    trapDoor.retract();
    chassis.waitUntil(20);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(0,0, 3000,{.forwards = false}, true);
    chassis.moveToPoint(-14, 14, 3000, {.forwards = false, .minSpeed = 15, .earlyExitRange = 2}, false);
    midgoal_first = true;
    trapDoor.extend();
    chassis.tank(-45, -45);
    topMotor.move_voltage(9000);
    intakeMotor.move_voltage(12000);
    pros::delay(1300);
    chassis.tank(0,0);
    intake_stop();
    midgoal_first = false;
    chassis.moveToPoint(-46, 46.5, 3500, {});
    chassis.waitUntil(7);
    trapDoor.retract();
    chassis.waitUntilDone();
    chassis.turnToPoint(-72, 46.5, 3500, {}, false);
    distancePose pose = distanceReset(true);
    intake();
    chassis.moveToPoint(-59, 46.5, 1200, {.minSpeed = 40}, false);
    chassis.tank(60, 60);
    pros::delay(300);
    chassis.tank(0,0);
    chassis.turnToPoint(-22, 47.5, 3000, {.forwards = false}, false);
    distanceReset(true);
    chassis.moveToPoint(-22 - longgoal_offset - 3.5, 47.5, 3000, {.forwards = false, .minSpeed = 15}, false);
    color_sort_enable = true;
    score_longgoal_auton(12000, allianceColor, 1500);
    intake_stop();
    color_sort_enable = false;
    distanceReset(true);
    chassis.moveToPoint(-49, 47.5, 3000, {.earlyExitRange = 2});
    distanceReset(true);
    descore.retract();
    chassis.waitUntilDone();
    chassis.moveToPose(-22 + longgoal_offset + 3, 47.5 + 13.5, 270, 5000, {.forwards = false ,.lead = 0.3, .minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPose(-16, 57.5 + 12.5, 270, 3000, {.forwards = false, .lead = 0.3});
}

void elim_auton() {
    descore.extend();
    chassis.setPose(-44.77, -12.31, 90);
    trapDoor.extend();
    intake();
    chassis.moveToPose(-24, -24, 145, 3000, {}); //24, 21
    chassis.waitUntil(3);
    trapDoor.retract();
    chassis.waitUntil(21);
    matchload_state(true);
    chassis.waitUntilDone();
    matchload_state(false);
    chassis.turnToPoint(-46, -47, 2000, {.forwards = false});
    chassis.moveToPoint(-46, -47, 3500, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(7);
    matchload_state(true);
    matchload.extend();
    chassis.waitUntilDone();
    chassis.turnToPoint(-72, -46.5, 3500, {}, false);
    distancePose pose = distanceReset(true);
    intake();
    chassis.moveToPoint(-59.5, -46.5, 1500, {}, false);
    chassis.tank(45, 45);
    pros::delay(300);
    chassis.tank(0,0);
    chassis.turnToPoint(-22, -47.5, 3000, {.forwards = false}, false);
    distanceReset(true);
    intake_stop();
    chassis.moveToPoint(-22 - longgoal_offset - 3.5, -47.5, 3000, {.forwards = false, .minSpeed = 15, .earlyExitRange = 1.5}, false);
    color_sort_enable = true;
    score_longgoal_auton(12000, allianceColor, 2400);
    intake_stop();
    color_sort_enable = false;
    matchload_state(false);
    intake_stop();
    distanceReset(true);
    descore.retract();
    chassis.moveToPoint(-48, -47.5, 3000, {}, false);
    chassis.moveToPose(-22 + longgoal_offset + 3, -47.5 +13.5, 270, 5000, {.forwards = false , .lead = 0.2, .minSpeed = 30, .earlyExitRange = 8});
    chassis.moveToPose(-16, -47.5 + 12.5, 270, 3000, {.forwards = false, .lead = 0.3});
}

void awp_auton() {
    color_sort_enable = true;
    descore.extend();
    chassis.setPose(-51.8, -17.8, 180);
    trapDoor.extend();
    matchload_state(true);
    chassis.moveToPoint(-51.8, -46.5, 1500, {});
    chassis.waitUntil(2);
    trapDoor.retract();
    chassis.waitUntilDone();
    distanceReset(false, false, true, false, true);
    chassis.turnToPoint(-72, -47, 1500, {});
    intake();
    chassis.moveToPoint(-61.7, -47, 1500, {}, false);
    chassis.tank(60, 60);
    pros::delay(matchload_delay - 240);
    chassis.tank(0,0);
    distanceReset(true);
    chassis.moveToPoint(-22  - longgoal_offset - 3, -47.5, 2000, {.forwards = false}, false);
    score_longgoal_auton(12000, allianceColor, 1250 - 100);
    distanceReset(true, false, false, false, true);
    chassis.moveToPoint(-36, -47.5, 2000, {}, false);
    intake();
    chassis.turnToPoint(-22, -22, 2000, {.minSpeed = 30, .earlyExitRange = 15});
    matchload_state(false);
    chassis.waitUntilDone();
    ltv.followPath(awp_1, {});
    ltv.waitUntil(16);
    matchload_state(true);
    ltv.waitUntil(26);
    matchload_state(false);
    ltv.waitUntil(55);
    matchload_state(true);

    ltv.waitUntilDone();
    std::cout << chassis.getPose().x << ", " << chassis.getPose().y << std::endl;
    pros::delay(20);
    distanceReset(false, true, true, false, true);
    pros::delay(20);
    chassis.moveToPoint(-22 - longgoal_offset - 4.7, 47.2, 1000, {.forwards = false}, false);
    score_longgoal_auton(12000, allianceColor, 1400);
    distanceReset(true);
    intake();
    chassis.moveToPoint(-60, 47, 2000, {}, false);
    chassis.tank(60, 60);
    pros::delay(500);
    chassis.tank(0,0);
    pros::delay(20);
    distanceReset(true);
    chassis.moveToPoint(-14, 14.5, 2000, {.forwards = false, .maxSpeed = 105}, false);
    trapDoor.extend();
    pros::delay(100);
    intakeMotor.move_voltage(9000);
    topMotor.move_voltage(11000);
    chassis.tank(-45, -45);
    pros::delay(1000);
    chassis.tank(0,0);

}

void skills_auton() {
    descore.extend();
    color_sort_enable = false;
    blockBlocker.retract();
    int start;
    distancePose pose;

    const int longgoal_delay = 2250;
    const int midgoal_delay = 1500;
    const int matchload_delay = 1250;
    const int mid_triball_delay = 500;
    const int longgoal_offset = 7;
    const int midgoal_offset = 11.7;
    const int matchload_offset = 11.4 + 0.7;
    const int triball_delay = 500;
    const int dual_ball_delay = 500;

    chassis.setPose(-53, 0, 270);
    intake();
    pros::delay(250);
    chassis.tank(100,100);
    pros::delay(500);
    chassis.tank(50,50);
    pros::delay(800);
    chassis.tank(-45,-45);
    pros::delay(400);
    chassis.tank(30,30);
    pros::delay(300);
    chassis.tank(0,0);
    pros::delay(700);
    int base_sum = 0;
    for(int i = 0; i < 5; i++)
    {
        base_sum += imu.get_roll();
        pros::delay(10);
    }
    int base = base_sum / 5;
    intake_stop();
    chassis.tank(-100, -100);
    start = pros::millis();
    while(imu.get_roll() < base + 5.5 && (pros::millis() - start < 2000))
    {
        pros::delay(10);
    }
    start = pros::millis();
    while(imu.get_roll() > base && (pros::millis() - start < 1500))
    {
        pros::delay(10);
    }
    chassis.tank(50,50);
    pros::delay(700);
    intakeFunnel.retract();
    intake_stop();
    chassis.setPose(-45.7, 0, chassis.getPose().theta);
    pose = distanceReset(true);
    pros::delay(20);
    chassis.moveToPoint(-45.7 + 5, 0, 2000, {.forwards = false}, false);
    chassis.turnToPoint(0,0, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    intakeFunnel.extend();
    pros::delay(20);
    distanceReset(true);
    chassis.moveToPose(-22.4, 22.4, 320, 3000, {.lead = 0.5});
    chassis.waitUntil(15);
    intake();
    chassis.waitUntilDone();
    intake_stop();
    chassis.moveToPose(-14, 14.5, 320, 700, {.forwards = false, .lead = 0.2}, false);
    std::cout << "Pose before midgoal: " << chassis.getPose().x << ", " << chassis.getPose().y << ", " << chassis.getPose().theta << std::endl;

    midgoal_first = true;
    color_sort_enable = false;
    trapDoor.extend();
    score_midgoal_auton();
    matchload_state(true);
    pros::delay(midgoal_delay);
    matchload_state(true);
    matchload.extend();
    intake_stop();
    topMotor.brake();
    pros::delay(500);

    chassis.moveToPoint(-45, 47.5, 2000, {});
    chassis.waitUntil(3);
    trapDoor.retract();
    color_sort_enable = false;
    chassis.waitUntilDone();
    chassis.turnToPoint(-72, 46.5, 2000, {}, false);
    distanceReset(true);
    chassis.moveToPoint(-72 + matchload_offset+1, 46.5, 2000, {}, false);
    intake(10000);
    chassis.tank(55,55);
    pros::delay(1000);
    intake(12000);
    pros::delay(matchload_delay + 170);
    chassis.tank(0,0);
    pros::delay(20);
    distanceReset(true);
    intake_stop();
    
    chassis.moveToPose(-22 + longgoal_offset, 61.5, 270, 1500, {.forwards = false, .lead = 0.3, .minSpeed = 60, .earlyExitRange = 7});
    chassis.moveToPose(37, 61.5, 270, 1700, {.forwards = false, .lead = 0, .maxSpeed = 100});
    chassis.waitUntil(3);
    matchload_state(false);
    chassis.waitUntilDone();
    pros::delay(20);
    distanceReset(true);
    chassis.swingToPoint(22, 50, lemlib::DriveSide::LEFT, 2500, {.forwards = false});
    chassis.moveToPoint(22 + longgoal_offset + 2, 50, 2000, {.forwards = false}, false);

    //alignToGoal(90);
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);
    matchload_state(true);
    matchload.extend();
    distanceReset(true);
    pros::delay(20);
    intake_stop();
    matchload_state(true);
    intake();
    chassis.moveToPoint(72 - matchload_offset-1, 46.5,  2000, {.forwards = true}, false);
    chassis.tank(55,55);
    pros::delay(matchload_delay + 600);
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
    chassis.moveToPoint(72 - matchload_offset, -47, 1000, {}, false);
    chassis.tank(60, 60);
    pros::delay(matchload_delay + 300);
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
    chassis.swingToPoint(-22, -51, lemlib::DriveSide::LEFT, 2000, {.forwards = false}, false);
    chassis.moveToPoint(-22 - longgoal_offset-2, -51, 1000, {.forwards = false, .minSpeed = 20}, false);
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);
    matchload_state(true);
    pros::delay(20);
    distanceReset(true);
    pros::delay(20);
    intake();
    chassis.moveToPoint(-72 + matchload_offset+2.5, -46.5, 2000, {}, false);
    chassis.tank(55,55);
    pros::delay(matchload_delay + 300);
    chassis.tank(0,0);
    chassis.turnToPoint(-22 - longgoal_offset, -47.5, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset - 2, -47.5, 2000, {.forwards = false, .minSpeed = 20}, false);
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

void test_auton()
{
    chassis.setPose(-51.25, -18.5, 180);
    distanceReset(true);
}

