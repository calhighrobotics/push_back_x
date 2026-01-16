#include "globals.h" 
#include "auton/autonFunctions.h"
#include "pros/rtos.hpp"
#include "velocityController.h"
#include "paths.h"
#include "ramsete.h"
#include "distanceReset.h"
#include "MCL.h"
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
5.8432642308,
0.213526937516,
1.14429410811,
0.906356177095,
0.347072436421,
11.4953431776,
54.5797495382,
};

    
RamsetePathFollower ramsete(config,2, 0.9);
/*
class Vector2 {
 public:
     Vector2(float x, float y) : x(x), y(y) {}
     std::string latex() const {
         std::ostringstream oss;
         oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
         return oss.str();
     }

     float x;
     float y;
};

const float INCH_TO_METER = 0.0254f;
const float TRACK_WIDTH = 11.5f;

const float wheel_circumference = lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
const float gear_ratio = 4.0f / 3.0f;

const float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;

void velocity_test(const VelocityControllerConfig &config, float max_velocity, int duration, int acceleration_time) {
    duration /= 10;
    acceleration_time /= 10;

    VoltageController controller(config.kV, config.KA_straight, config.KA_turn, config.KS_straight, config.KS_turn, config.KP_straight, config.KI_straight, 99999, 11.5);

    std::cout << "\\left[";

    int i;
    for (i = 0; i < duration; ++i) {
        auto v_d = max_velocity * fminf(fminf(1, (float)i / (float)acceleration_time),
                                        (float)(duration - i) / (float)acceleration_time);
        auto speed = (leftMotors.get_actual_velocity() + rightMotors.get_actual_velocity()) / 2;
        std::cout << Vector2(i * 0.01f, speed).latex() << ",";
        std::cout.flush();
        auto voltage = controller.update(v_d, 0, leftMotors.get_actual_velocity() * rpm_to_mps_factor, rightMotors.get_actual_velocity() * rpm_to_mps_factor);
        leftMotors.move_voltage(voltage.leftVoltage * 1000);
        rightMotors.move_voltage(voltage.rightVoltage * 1000);

        pros::delay(10);
    }
    std::cout << Vector2(0.01f * (float)i, (leftMotors.get_actual_velocity() + rightMotors.get_actual_velocity()) / 2)
                     .latex()
              << "\\right]" << std::endl;

    leftMotors.brake();
    rightMotors.brake();
    std::cout << "\b" << std::endl;
}
*/
void precompute_auton_paths() {
    std::vector<std::string> paths = {};
    ramsete.precompute_paths(paths);
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
    chassis.moveToPoint(-22 - longgoal_offset - 7, longgoal_y_believed, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset - 1, longgoal_y_believed, 1000, {.forwards = false});
    score_longgoal_auton();

}

void carry_auton() {
    //chassis.setPose(-24, 24, 90);
}

void left_auton() {

    chassis.setPose(-48.147, 10.854, 70);
    intake();
    chassis.moveToPoint(-22, 24, 1000);
    chassis.waitUntilDone();
    matchload_state(true);
    pros::delay(triball_delay);
    matchload_state(false);
    matchload_state(true);
    pros::delay(dual_ball_delay);
    matchload_state(false);

    chassis.waitUntilDone();
    resting_state();
    score_midgoal(12000);

    pros::delay(midgoal_delay); 
    chassis.turnToPoint(-48, 47, 750);
    chassis.moveToPoint(-48, 47, 1500);

    //Intake from matchload
    chassis.turnToPoint(-72, 47, 750);
    chassis.waitUntilDone();
    matchload_state(true);
    chassis.moveToPoint(-72 + matchload_offset, 47, 1000);
    chassis.waitUntilDone();
    intake();
    pros::delay(matchload_delay);

    //Score on longgoal

    chassis.moveToPoint(-48 - longgoal_offset,  47, 1000, {.forwards = false});
    chassis.waitUntilDone();
    resting_state();
    score_longgoal();
    
}

void elim_auton() {
    crossBarrier();
}

void awp_auton() {
    /*
    MCL::StartMCL(-51.25, -18.5, 180);
    pros::Task mclTask(MCL::MonteCarlo);
    enable_fused_odometry(true);
    */
    //colorSort(Color::RED);
    //FIRST LONGGOAL
    /*
    MCL::StartMCL(-51.25, -18.5, 180);
    pros::Task mclTask(MCL::MonteCarlo);
    enable_fused_odometry(true);
    */
    //colorSort(Color::RED);

    //FIRST LONGGOAL
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
    
    /*
    matchload_state(true);
    intake();
    ramsete.followPath(awp_1, {.end_correction = true};
    */

    //pros::delay(matchload_delay);
    chassis.turnToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset, -51.5, 1000, {.forwards = false, .minSpeed=35});
    chassis.waitUntilDone();
    resting_state();
    matchload_state(false);
    score_longgoal_auton();
    distanceReset(true);
    pros::delay(longgoal_delay + 150);
    resting_state();
    relativeMotion(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, 4, 200, true);

    //MIDDLE AUTON
    //chassis.swingToPoint(-20 , -24, lemlib::DriveSide::RIGHT, 800);
    chassis.turnToPoint(-24, -24, 1000);

    intake();
    chassis.moveToPoint(-24 , -24, 1000, {.maxSpeed = 85, .minSpeed = 20});
    chassis.waitUntil(17);
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
    //relativeMotion(-20, 20, 133, 11.5, 1000, false);
    //chassis.moveToPoint(-10, 11.7, 1000, {.forwards = false});
    chassis.moveToPoint(-12.3, 14.2, 1000, {.forwards = false});
    //chassis.moveToPose(-11, 12, 133, 850, {.forwards = false, .minSpeed = 20});
    chassis.waitUntilDone();
    resting_state();
    score_midgoal();
    pros::delay(midgoal_delay + 150);
    intake_stop();
    matchload_state(true);
    intake();
    chassis.turnToPoint(-45, 54, 500);
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

    const int longgoal_delay = 1100;
    const int midgoal_delay = 1000;
    const int matchload_delay = 500;
    const int mid_triball_delay = 500;
    const int longgoal_offset = 6;
    const int midgoal_offset = 11.7;
    const int matchload_offset = 11.9;
    const int triball_delay = 500;
    const int dual_ball_delay = 500;
    const float starting_pitch = imu.get_pitch();


    chassis.setPose(-55, 0, 270);
    chassis.tank(90, 90);
    pros::delay(1200);
    leftMotors.brake();
    rightMotors.brake();
    pros::delay(1000);
    chassis.tank(-100, -100);
    pros::delay(1000);
    leftMotors.brake();
    rightMotors.brake();
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    chassis.setPose(-46, 0, 270);
    distancePose pose = distanceReset(true);
    std::cout << "Believed Y from parking zone" << pose.y << std::endl;
    chassis.setPose(pose.x, pose.y, 270);
    chassis.turnToPoint(-24, 24, 1000);
    chassis.moveToPoint(-24, 24, 2000);
    chassis.turnToPoint(-12.3, 14.2, 1000, {.forwards = false});
    chassis.moveToPoint(-12.3, 14.2, 1500, {.forwards = false}, false);
    
    chassis.turnToPoint(-45, 54, 1000);
    chassis.moveToPoint(-45, 54, 2000);
    chassis.turnToPoint(-72 + matchload_offset, 47.5, 1000, {}), false;
    distanceReset(true);
    chassis.moveToPoint(-72 + matchload_offset, 47.5, 1500, {}, false);
    ramsete.followPath(skills_1, {.backwards = true});
    chassis.moveToPoint(22 + longgoal_offset, 62, 3000, {.forwards = false, .maxSpeed = 90}, false);
    distanceReset(true);
    chassis.turnToPoint(45, 48, 1500, {.forwards = false});
    chassis.moveToPoint(47, 48, 2000, {.forwards = false});
    chassis.turnToPoint(22 + longgoal_offset - 2, 48, 1000, {.forwards = false}, false);
    distanceReset(true);
    chassis.moveToPoint(22 + longgoal_offset - 2, 48, 1500, {.forwards = false}, false);
    distanceReset(true);
    chassis.turnToPoint(72 - matchload_offset, 47, 1000);
    chassis.moveToPoint(72 - matchload_offset, 47, 2000, {}, false);
    pros::delay(matchload_delay);
    chassis.turnToPoint(22 + longgoal_offset - 2, 48, 1000, {.forwards = false});
    chassis.moveToPoint(22 + longgoal_offset - 2, 48, 1500, {.forwards = false}, false);
    pros::delay(longgoal_delay);
    distanceReset(true);
    ramsete.followPath(skills_2, {.backwards = false, .end_correction = true});
    chassis.tank(100, 100);
    pros::delay(1500);
    chassis.turnToHeading(180, 2000);
    chassis.waitUntilDone();
    distanceReset(true);
    ramsete.followPath(skills_3, {.backwards = false, .end_correction = true});
    chassis.turnToPoint(72 - matchload_offset, -47, 1000);
    distanceReset(true);
    chassis.moveToPoint(72 - matchload_offset, -47, 2000, {}, false);
    pros::delay(matchload_delay);




}