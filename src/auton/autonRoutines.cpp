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
#include "colorSort.h"

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

const int longgoal_delay = 4000;
const int midgoal_delay = 3000;
const int matchload_delay = 100;
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

    
RamsetePathFollower ramsete(config,1, 0.9);
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
    std::vector<std::string> paths = {right_1, right_2, left_1, skills_1};
    ramsete.precompute_paths(paths);
}

void right_auton()
{
    chassis.setPose(-48, -10.5, 115);
    intake(12000);
    chassis.turnToPoint(-22, -22, 1000);
    chassis.moveToPoint(-22, -22, 1000);
    chassis.waitUntilDone(); 
    matchload_state(true);
    pros::delay(triball_delay);
    matchload_state(false);
    //Ramsete path to dual_ball
    chassis.turnToPoint(-6, -44, 1000);
    chassis.moveToPoint(-6, -44,  1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    matchload_state(true);

    pros::delay(dual_ball_delay + 2000);

    matchload_state(false);

    //Ramsete backward path to -47, -47
    ramsete.followPath(right_2, {.backwards = true, .log = true, .path_index = 2});

    intake_stop();
    chassis.turnToPoint(-72, -48, 1000);
    matchload_state(true);
    chassis.moveToPoint(-72 + matchload_offset, -48, 1000);
    intake();
    pros::delay(matchload_delay + 1000);
    chassis.moveToPoint(-31, -48, 1000, {.forwards = false});
    chassis.waitUntilDone();
    matchload_state(false);
    intake_stop();
    score_longgoal();
}

void carry_auton() {
    //chassis.setPose(-24, 24, 90);
    ramsete.followPath(test_path, {.log = true, .test = true});
}

void left_auton() {

    chassis.setPose(-48.147, 10.854, 75);
    intake();
    ramsete.followPath(left_1, {.path_index = 3});

    pros::delay(dual_ball_delay);

    chassis.moveToPose(-24, 24, 90, 2000, {.forwards = false});

    //Score on midgoal
    chassis.turnToPoint(-11.7, 11.7, 750);
    chassis.moveToPoint(-11.7, 11.7, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake_stop();
    score_midgoal(12000);

    pros::delay(midgoal_delay); 
    chassis.turnToPoint(-48, 48, 750);
    chassis.moveToPoint(-48, 48, 1500);

    //Intake from matchload
    chassis.turnToPoint(-72, 48, 750);
    chassis.waitUntilDone();
    matchload_state(true);
    chassis.moveToPoint(-72 + matchload_offset, 48, 1000);
    chassis.waitUntilDone();
    intake();
    pros::delay(matchload_delay);

    //Score on longgoal

    chassis.moveToPoint(-48 - longgoal_offset,  48, 1000, {.forwards = false});
    chassis.waitUntilDone();
    matchload_state(false);
    intake_stop();
    score_longgoal();
    
}

void elim_auton() {}

void awp_auton() {
    /*
    MCL::StartMCL(-51.25, -18.5, 180);
    pros::Task mclTask(MCL::MonteCarlo);
    enable_fused_odometry(true);
    */
    //colorSort(Color::RED);
    //FIRST LONGGOAL
    chassis.setPose(-51.25, -18.5, 180);
    chassis.moveToPoint(-51.25, -51, 1500);
    chassis.waitUntilDone();

    chassis.turnToPoint(-72, -48, 750);
    matchload_state(true);
    chassis.waitUntilDone();
    //distanceReset();
    pros::delay(500);
    intake();
    chassis.moveToPoint(-72 + matchload_offset, -48, 1000);
    chassis.waitUntilDone();
    //matchload_wiggle(matchload_delay);
    pros::delay(matchload_delay);

    chassis.moveToPoint(-24 - longgoal_offset, -51, 1500, {.forwards = false, .maxSpeed = 80});
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
    pros::delay(longgoal_delay);
    intake_stop();

    //MIDDLE AUTON
    chassis.swingToPoint(-20 , -24, lemlib::DriveSide::RIGHT, 1000);
    chassis.turnToPoint(-20, -24, 500);
    intake();
    chassis.moveToPoint(-20 , -24, 1000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    matchload_state(true);
    pros::delay(mid_triball_delay);
    chassis.turnToPoint(-20, 29, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-20, 29, 2000, {.maxSpeed = 60});
    chassis.waitUntil(5);
    matchload_state(false);
    chassis.waitUntilDone();
    matchload_state(true);
    pros::delay(mid_triball_delay);
    chassis.waitUntilDone();
    chassis.moveToPoint(-20, 26, 750, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToPoint(0, 0, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    relativeMotion(-20, 26, 135, 15, 1500, false);
    chassis.waitUntilDone();
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_midgoal();
    pros::delay(midgoal_delay);
    intake_stop();

    //2nd LONGGOAL
    chassis.moveToPoint(-48, 51, 1500);
    matchload_state(true);
    chassis.turnToPoint(-72, 51, 1000);
    chassis.moveToPoint(-72 + matchload_offset, 51, 1500);
    intake();
    chassis.waitUntilDone();
    pros::delay(matchload_delay);

    chassis.moveToPoint(-24 - longgoal_offset + 4, 51, 2000, {.forwards = false});
    chassis.waitUntil(5);
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
}

void skills_auton() {

    chassis.setPose(-49.25,-18.5,180);
    intake();
    chassis.moveToPoint(-53.713, 47.034, 1000);
    chassis.turnToPoint(-60.114, 47.034, 500);
    chassis.waitUntilDone();
    matchload_state(true);
    pros::delay(matchload_delay);

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
    pros::delay(matchload_delay);

    chassis.moveToPoint(29.5, 47, 1000, {.forwards = false});
    matchload_state(false);
    chassis.waitUntilDone();
    intake_stop();
    score_longgoal();
    pros::delay(longgoal_delay);
}