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
    int longgoal_y_believed = -48;

    chassis.setPose(-51.25, -18.5, 180);
    intake();
    chassis.moveToPoint(-51.25, -51, 1000);
    matchload_state(true);
    chassis.waitUntilDone();

    chassis.turnToPoint(-72, -51, 1000);
    chassis.waitUntilDone();
    
    chassis.moveToPoint(-72 + matchload_offset - 1.25, -51, 1000, {.minSpeed = 20});
    chassis.waitUntilDone();
    pros::delay(175);
    chassis.turnToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false, .minSpeed=35});
    chassis.waitUntilDone();
    resting_state();
    matchload_state(false);
    score_longgoal_auton();
    distancePose pose = distanceReset(true);
    longgoal_y_believed = pose.y;
    pros::delay(longgoal_delay + 150);
    resting_state();
    relativeMotion(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, 4, 200, true);
    chassis.turnToPoint(-22, -22, 1000);
    intake();
    chassis.moveToPoint(-22 , -22, 1000, {.maxSpeed = 85, .minSpeed = 20});
    chassis.waitUntil(16);
    matchload_state(true);
    chassis.waitUntilDone();

    chassis.turnToPoint(-39, longgoal_y_believed, 1000, {.forwards = false});
    chassis.moveToPoint(-39, longgoal_y_believed, 1500, {.forwards = false});
    chassis.turnToPoint(-22 - longgoal_offset - 2, longgoal_y_believed, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset - 2, longgoal_y_believed, 1500, {.forwards = false}, false);
    intake_stop();
    score_longgoal_auton();
    pros::delay(1000);
    resting_state(false);
    chassis.moveToPoint(-22 - longgoal_offset - 1, longgoal_y_believed, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset - 1, longgoal_y_believed, 1000, {.forwards = false});
    score_longgoal_auton();
    pros::delay(1000);


}

void carry_auton() {
    chassis.setPose(-51.25, -18.5, 180);
    distanceReset(true);
    intake();
    pros::delay(matchload_delay);
    chassis.turnToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false, .minSpeed=35}, false);
    intake_stop();
    score_longgoal_auton();
    pros::delay(longgoal_delay + 150);
    matchload_state(false);
    distanceReset(true);

    
}

void left_auton() {
    const int longgoal_delay = 1100;
    const int midgoal_delay = 1000;
    const int matchload_delay = 500;
    const int mid_triball_delay = 500;
    const int longgoal_offset = 6 - 2;
    const int midgoal_offset = 11.7;
    const int matchload_offset = 11.9;
    const int triball_delay = 500;
    const int dual_ball_delay = 500;

    chassis.setPose(-47.539, 11.5, 67.4);
    ltv.followPath(left_1, {});
    intake();
    matchload_state(true); 
    ltv.followPath(left_2, {.backwards = true});
    intake_stop();
    score_midgoal();
    pros::delay(midgoal_delay);
    intake_stop();
    
    matchload_state(true);
    intake();
    ltv.followPath(left_3, {});
    pros::delay(matchload_delay);

    distanceReset(true);
    chassis.turnToPoint(-22 - longgoal_offset, 47.5, 2000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset, 47.5, 2000, {.forwards = false}, false);
    score_longgoal_auton();
    intake_stop();
    pros::delay(longgoal_offset);

    ltv.followPath(left_4, {});
    chassis.turnToPoint(-13, 48.5, 2000, {.forwards = false});
    chassis.moveToPoint(-13, 48.5, 2000, {.forwards = false, .maxSpeed = 85});

    leftMotors.set_brake_mode(pros::MotorBrake::hold);
    rightMotors.set_brake_mode(pros::MotorBrake::hold);
    leftMotors.brake();
    rightMotors.brake();






}

void elim_auton() {
    chassis.setPose(22 + longgoal_offset-1, 47.5, 90);
    distanceReset(true);
    std::cout << "Pose after distance reset: " << chassis.getPose().x << ", " << chassis.getPose().y << std::endl;
    intake_stop();
    matchload_state(true);
    intake();
    chassis.moveToPoint(72 - matchload_offset, 46.5, 3000, {.forwards = true, .maxSpeed = 70}, false);
    chassis.tank(45,45);
    pros::delay(matchload_delay + 1000);
    chassis.tank(0,0);
    chassis.turnToPoint(22 + longgoal_offset, 47.5, 1000, {.forwards = false}, false);
    chassis.moveToPoint(22 + longgoal_offset+2, 47.5, 2000, {.forwards = false, .minSpeed = 25}, false);
    score_longgoal_auton();
    pros::delay(longgoal_delay + 150);

    distanceReset(true);
    pros::delay(20);
    /*
    relativeMotion(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, 6, 200, true);
    chassis.moveToPose(43, -2, 180, 2000, {.lead = 0.2}, false);
    chassis.turnToPoint(72, -5, 1000, {}, false);
    matchload_state(false);
    intake();
    pros::delay(1000);
    crossBarrier(1, false, false);
    pros::delay(250);
    chassis.tank(60,60);
    pros::delay(1500);
    chassis.tank(0,0);
    pros::delay(2000);
    intake_stop();
    crossBarrier(1, true, true);
    chassis.tank(0,0);
    chassis.turnToHeading(90, 2000);
    distanceReset(true);

    chassis.moveToPose(22, 22, 225, 3000, {.forwards = false});
    intakeFunnel.retract();
    chassis.moveToPose(12.5, 10.5, 225, 3000, {}, false);
    intake();
    pros::delay(500);
    basket.extend();
    outtake(8000);
    pros::delay(600);
    outtake(10000);
    pros::delay(midgoal_delay + 1000);
    intake_stop();
    basket.retract();

    ltv.followPath(skills_6, {.backwards = true, .q_x = 800, .r_ang = 95});
    ltv.waitUntil(10);
    intake();
    ltv.waitUntilDone();
    distanceReset(false, false, false, true, true);
    chassis.moveToPoint(chassis.getPose().x, -46.5, 2000, {});
    chassis.turnToPoint(72, -46.5, 1000, {}, false);
    distanceReset(true);
    */
}

void awp_auton() {
    descore.extend();
    chassis.setPose(-51.25, -18.5, 180);
    intake();
    chassis.moveToPoint(-51.25, -51, 1000);
    matchload_state(true);
    chassis.waitUntilDone();

    chassis.turnToPoint(-72, -51, 1000);
    chassis.waitUntilDone();
    
    chassis.moveToPoint(-72 + matchload_offset, -51, 1000, {.minSpeed = 20});
    chassis.waitUntilDone();
    pros::delay(50);
    

    chassis.turnToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false, .minSpeed=35});
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

    chassis.moveToPoint(-22 - longgoal_offset - 2.5, 47.5, 2000, {.forwards = false, .minSpeed = 35});
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
    const int midgoal_delay = 2250;
    const int matchload_delay = 1000;
    const int mid_triball_delay = 500;
    const int longgoal_offset = 7;
    const int midgoal_offset = 11.7;
    const int matchload_offset = 9.9;
    const int triball_delay = 500;
    const int dual_ball_delay = 500;

    if(pros::battery::get_capacity() <= 25)
    {   
        controller.rumble("...");
        pros::delay(1000);
    }

    chassis.setPose(-53, 0, 270);
    intake();
    pros::delay(250);
    chassis.tank(50,50);
    pros::delay(1200);
    chassis.tank(0,0);
    pros::delay(2000);

    intake_stop();
    chassis.tank(-100, -100);
    start = pros::millis();
    while(imu.get_roll() < 6 && (pros::millis() - start < 2000))
    {
        pros::delay(10);
    }
    start = pros::millis();
    while(imu.get_roll() > 0.5 && (pros::millis() - start < 1500))
    {
        pros::delay(10);
    }
    chassis.tank(0,0);
    chassis.tank(40, 40);
    pros::delay(950);
    chassis.tank(0,0);
    intakeFunnel.retract();
    intake_stop();
    chassis.setPose(-45.7, -0.5, 270);
    distancePose pose = distanceReset(true);
    pros::delay(20);
    chassis.moveToPoint(pose.x + 5, 0, 2000, {.forwards = false}, false);
    chassis.turnToPoint(0,0, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    intakeFunnel.extend();
    distanceReset(true);
    pros::delay(20);
    ltv.followPath(skills_1, {.q_x = 950, .r_ang = 90});
    ltv.waitUntil(3);
    intake_stop();
    ltv.waitUntil(25);
    intake();
    ltv.waitUntil(ltv.getPathLength(skills_1) - 2.2);
    intake_stop();
    ltv.waitUntilDone();

    chassis.turnToPoint(0,0, 1000, {.forwards = false}, false);
    chassis.moveToPose(-12, 17.2, 316, 1500, {.forwards = false, .lead = 0.2, .maxSpeed = 75}, false);
    std::cout << "Pose before midgoal: " << chassis.getPose().x << ", " << chassis.getPose().y << ", " << chassis.getPose().theta << std::endl;

    midgoal_first = true;
    color_sort_enable = false;
    trapDoor.extend();
    pros::delay(200);
    score_midgoal_auton();
    pros::delay(300);
    matchload_state(true);
    pros::delay(midgoal_delay);
    intake_stop();
    topMotor.brake();
    chassis.tank(0,0);
    pros::delay(500);

    chassis.moveToPoint(-46, 50, 2000, {.earlyExitRange = 4});
    chassis.waitUntil(3);
    trapDoor.retract();
    color_sort_enable = false;
    chassis.waitUntilDone();
    chassis.turnToPoint(-72, 49, 2000, {}, false);
    distanceReset(true);
    chassis.moveToPoint(-72 + matchload_offset+1, 46.75, 3000, {}, false);
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
    ltv.followPath(skills_3, {.backwards = true, .q_x = 1500, .q_y = 100000});
    ltv.waitUntil(3);
    matchload_state(false);
    ltv.waitUntil(ltv.getPathLength(skills_3) - 3);
    ltv.cancel();
    distanceReset(true);
    chassis.swingToPoint(-20, 51.5, lemlib::DriveSide::LEFT,3000, {.forwards = false, .earlyExitRange = 3});
    chassis.moveToPose(-22 - longgoal_offset, 49.5, 90, 1500, {.forwards = false, .lead = 0.7, .minSpeed = 20}, false);
    score_longgoal_auton();
    matchload_state(true);
    matchload.extend(); 
    pros::delay(longgoal_delay + 150);

    //Excise
    distanceReset(true);
    std::cout << "Pose after distance reset: " << chassis.getPose().x << ", " << chassis.getPose().y << std::endl;
    intake_stop();
    matchload_state(true);
    intake();
    chassis.moveToPoint(72 - matchload_offset + 1, 46.5, 3000, {.forwards = true, .maxSpeed = 70}, false);
    chassis.tank(45,45);
    pros::delay(matchload_delay);
    chassis.tank(0,0);
    chassis.turnToPoint(22 + longgoal_offset, 47.5, 1000, {.forwards = false}, false);
    chassis.moveToPoint(22 + longgoal_offset+2, 47.5, 2000, {.forwards = false, .minSpeed = 25});
    chassis.waitUntil(3);
    intake_stop();
    chassis.waitUntilDone();
    score_longgoal_auton();
    pros::delay(longgoal_delay + 150);

    distanceReset(true);
    pros::delay(20);
    // Excise
    relativeMotion(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, 6, 200, true);
    chassis.turnToPoint(48, -46.5, 2000, {}, false);
    distanceReset(false, false, false, true, true);
    chassis.moveToPoint(48, -43.5, 6000, {.maxSpeed = 95}, false);
    matchload_state(true);
    distanceReset(false, false, true, false, true);
    chassis.turnToPoint(72 - matchload_offset + 1, -43.5, 3000, {}, false);
    distanceReset(true);

    intake();
    chassis.moveToPoint(72 - matchload_offset + 1.5, -46.5, 1000, {}, false);
    chassis.tank(45, 45);
    pros::delay(matchload_delay + 1000 + 300);
    chassis.tank(0,0);
    distanceReset(true);
    pros::delay(20);

    ltv.followPath(skills_7, {.backwards = true, .q_x = 1500, .q_y = 100000});
    ltv.waitUntil(3);
    intake_stop();
    ltv.waitUntil(10);
    matchload_state(false);
    ltv.waitUntil(ltv.getPathLength(skills_7) - 3);
    ltv.cancel();
    distanceReset(true);
    chassis.swingToPoint(-22, -50.5, lemlib::DriveSide::LEFT,3000, {.forwards = false, .earlyExitRange = 1});
    chassis.moveToPose(-22 - longgoal_offset - 2.5, -49.5, 270, 2000, {.forwards = false, .lead = 0.7, .minSpeed = 20, }, false);
    score_longgoal_auton();
    pros::delay(longgoal_delay);
    distanceReset(true);
    pros::delay(20);
    matchload_state(true);
    chassis.turnToPoint(-72 + matchload_offset + 1, -46.5, 1000, {}, false);
    distanceReset(true);
    intake();
    chassis.moveToPoint(-72 + matchload_offset + 2, -46.5, 2000, {}, false);
    chassis.tank(45, 45);
    pros::delay(matchload_delay + 1500);
    chassis.tank(0,0);
    chassis.turnToPoint(-22 - longgoal_offset, -47.3, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset - 2, -47.3, 2000, {.forwards = false, .minSpeed = 20}, false);
    score_longgoal_auton();
    pros::delay(longgoal_delay + 1500);
    distanceReset(true);
    pros::delay(20);
    matchload_state(false);

    chassis.moveToPose(-64, -16.7, 0, 3000, {.lead = 0.5, .maxSpeed = 95}, false);
    intake();
    crossBarrier(1);
    chassis.tank(0,0);
    leftMotors.brake();
    rightMotors.brake();
    
}