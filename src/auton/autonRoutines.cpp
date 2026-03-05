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
    distancePose pose = distanceReset(true);
    intake();
    chassis.moveToPoint(-59.5, -46.5, 1500, {}, false);
    chassis.tank(45, 45);
    pros::delay(300);
    chassis.tank(0,0);
    chassis.turnToPoint(-22, -47.5, 3000, {.forwards = false}, false);
    distanceReset(true);
    intake_stop();
    chassis.moveToPoint(-22 - longgoal_offset - 3.5, -46.7, 3000, {.forwards = false, .minSpeed = 15, .earlyExitRange = 1.5}, false);
    color_sort_enable = true;
    score_longgoal_auton(12000, allianceColor, 1500);
    intake_stop();
    color_sort_enable = false;
    matchload_state(false);
    intake_stop();
    distanceReset(true);
    descore.retract();
    chassis.moveToPoint(-48, -46.5, 3000, {}, false);
    ltv.followPath(right_1, {.backwards = true, .turnFirst = true});
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
    pros::delay(1000);
    chassis.tank(0,0);
    intake_stop();
    midgoal_first = false;
    chassis.moveToPoint(-46, 46.5, 3500, {});
    chassis.waitUntil(7);
    trapDoor.retract();
    chassis.waitUntilDone();
    chassis.turnToPoint(-72, 46.7, 3500, {}, false);
    distancePose pose = distanceReset(true);
    intake();
    chassis.moveToPoint(-60, 46.5, 1200, {.minSpeed = 40}, false);
    chassis.tank(60, 60);
    pros::delay(400);
    chassis.tank(0,0);
    chassis.turnToPoint(-22, 47.5, 3000, {.forwards = false}, false);
    distanceReset(true);
    chassis.moveToPoint(-22 - longgoal_offset - 3.5, 47.5, 3000, {.forwards = false, .minSpeed = 15}, false);
    color_sort_enable = true;
    score_longgoal_auton(12000, allianceColor, 1500);
    intake_stop();
    color_sort_enable = false;
    distanceReset(true);
    chassis.moveToPoint(-48, 47.5, 3000, {.earlyExitRange = 2});
    descore.retract();
    chassis.waitUntilDone();
    ltv.followPath(left_2, {.backwards = true, .q_y_backward = 3500, .q_theta_backward = 40});
    ltv.waitUntil(ltv.getPathLength(left_2) - 1);
    ltv.cancel();
}

void elim_auton() {

}

void awp_auton() {
    descore.extend();
    chassis.setPose(-51.25, -18.5, 180);
    trapDoor.extend();
    pros::delay(250);
    trapDoor.retract();
    matchload_state(true);
    intake();
    chassis.moveToPoint(-51.25, -51, 1000);
    matchload_state(true);
    chassis.waitUntilDone();

    chassis.turnToPoint(-72, -51, 1000);
    chassis.waitUntilDone();
    
    chassis.moveToPoint(-72 + matchload_offset + 2, -51, 1000, {});
    chassis.waitUntilDone();
    chassis.tank(45, 45);
    pros::delay(50);
    chassis.tank(0,0);

    chassis.turnToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false, .minSpeed=20});
    chassis.waitUntilDone();
    resting_state();
    matchload_state(false);
    score_longgoal_auton();
    distanceReset(true);
    pros::delay(longgoal_delay + 150);
    resting_state();
    matchload_state(true);
    chassis.waitUntilDone();

    chassis.turnToPoint(-24, 24, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-24, 24, 1500, {.maxSpeed = 85});
    chassis.waitUntil(5);
    matchload_state(false);
    chassis.waitUntil(39);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-12.3, 14.2, 1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-12.3, 14.2, 1000, {.forwards = false});
    chassis.waitUntilDone();
    resting_state();
    score_midgoal();
    pros::delay(midgoal_delay + 150);
    intake_stop();
    matchload_state(true);
    intake();
    chassis.turnToPoint(-45, 54, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-45, 54, 1200);
    chassis.waitUntilDone();
    trapDoor.retract();
    matchload_state(true);
    matchload.extend();
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    pros::delay(5);
    distancePose pose = distanceReset(true);
    std::cout << "x: " << pose.x << " y: " << pose.y << std::endl;
    chassis.moveToPoint(-72 + matchload_offset - 2, 47.5, 1500, {.maxSpeed = 100, .minSpeed = 20});
    chassis.waitUntilDone();
    pros::delay(250);
    
    chassis.turnToPoint(-22 - longgoal_offset - 2.5, 47.5, 1000, {.forwards = false});

    chassis.moveToPoint(-22 - longgoal_offset - 2.5, 47.5, 2000, {.forwards = false, .minSpeed = 25});
    chassis.waitUntil(10);
    matchload_state(false);
    chassis.waitUntilDone();
    score_longgoal_auton();
}

void skills_auton() {
    descore.extend();
    color_sort_enable = false;
    blockBlocker.retract();
    int start;

    const int longgoal_delay = 2000;
    const int midgoal_delay = 1250;
    const int matchload_delay = 1000;
    const int mid_triball_delay = 500;
    const int longgoal_offset = 7;
    const int midgoal_offset = 11.7;
    const int matchload_offset = 11.9;
    const int triball_delay = 500;
    const int dual_ball_delay = 500;

    chassis.setPose(-53, 0, 270);
    intake();
    pros::delay(250);
    chassis.tank(50,50);
    pros::delay(1000);
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
    chassis.tank(40,40);
    pros::delay(700);
    intakeFunnel.retract();
    intake_stop();
    chassis.setPose(-45.7, 1.5, chassis.getPose().theta);
    distancePose pose = distanceReset(true);
    pros::delay(20);
    chassis.moveToPoint(-45.7 + 5, 0, 2000, {.forwards = false}, false);
    chassis.turnToPoint(0,0, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    intakeFunnel.extend();
    distanceReset(true);
    pros::delay(20);
    ltv.followPath(skills_1, {});
    ltv.waitUntil(3);
    intake_stop();
    ltv.waitUntil(25);
    intake();
    ltv.waitUntilDone();
    intake_stop();
    ltv.waitUntilDone();
    chassis.turnToPoint(0,0, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-14, 14, 3500, {.forwards = false}, false);
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
    chassis.turnToPoint(-72, 46.3, 2000, {}, false);
    distanceReset(true);
    chassis.moveToPoint(-72 + matchload_offset+2, 46.5, 3000, {}, false);
    intake(10000);
    chassis.tank(50,50);
    pros::delay(1000);
    intake(12000);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    pros::delay(10);
    distanceReset(true);
    pros::delay(2);

    intake_stop();
    ltv.followPath(skills_3, {.backwards = true});
    ltv.waitUntil(3);
    matchload_state(false);
    ltv.waitUntilDone();
    //alignToGoal(90);
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);
    matchload_state(true);
    matchload.extend();
    distanceReset(true);
    pros::delay(20);
    std::cout << "Pose after distance reset: " << chassis.getPose().x << ", " << chassis.getPose().y << std::endl;
    intake_stop();
    matchload_state(true);
    intake();
    chassis.turnToPoint(72, 46.5, 3000, {});
    chassis.moveToPoint(72 - matchload_offset-2, 46.5, 3000, {.forwards = true}, false);
    chassis.tank(45,45);
    pros::delay(matchload_delay + 600);
    chassis.tank(0,0);
    chassis.turnToPoint(22 + longgoal_offset, 46.5, 1000, {.forwards = false}, false);
    chassis.moveToPoint(22 + longgoal_offset+2, 46.5, 2000, {.forwards = false, .minSpeed = 25});
    chassis.waitUntil(3);
    intake_stop();
    chassis.waitUntilDone();
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);

    distanceReset(true);
    pros::delay(20);

    relativeMotion(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, 6, 200, true);
    chassis.turnToPoint(45, -46.5, 2000, {}, false);
    distanceReset(false, false, false, true, true);
    chassis.moveToPoint(45, -46.5, 6000, {.maxSpeed = 95}, false);
    matchload_state(true);
    distanceReset(false, false, true, false, true);
    chassis.turnToPoint(72, -46.5, 3000, {}, false);
    distanceReset(true);

    intake();
    chassis.moveToPoint(72 - matchload_offset-2, -46.8, 1000, {}, false);
    chassis.tank(45, 45);
    pros::delay(matchload_delay + 150);
    chassis.tank(0,0);
    distanceReset(true);
    pros::delay(20);

    ltv.followPath(skills_7, {.backwards = true});
    ltv.waitUntil(3);
    intake_stop();
    ltv.waitUntil(10);
    matchload_state(false);
    ltv.waitUntilDone();
    //alignToGoal(90);
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);
    matchload_state(true);
    distanceReset(true);
    pros::delay(20);
    chassis.turnToPoint(-72, -46.5, 1000, {}, false);
    distanceReset(true);
    intake();
    chassis.moveToPoint(-72 + matchload_offset+1, -46.8, 2000, {}, false);
    chassis.tank(45, 45);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    chassis.turnToPoint(-22 - longgoal_offset, -47, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset - 2, -47, 2000, {.forwards = false, .minSpeed = 20}, false);
    score_longgoal_auton(12000, Color::RED, longgoal_delay + 150);  
    distanceReset(true);
    pros::delay(20);
    matchload_state(false);

    chassis.moveToPose(-63, -15.7, 0, 3000, {.lead = 0.3}, false);
    intake();
    crossBarrier(1);
    chassis.tank(40,40);
    pros::delay(700);

    leftMotors.brake();
    rightMotors.brake();
}