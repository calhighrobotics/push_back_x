#include "lemlib/chassis/chassis.hpp"
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
    chassis.setPose(-48, -10.5, 115);
    intake(12000);
    chassis.moveToPoint(-22, -24,550);
    chassis.waitUntil(10.5); 
    matchload_state(true);
    pros::delay(100);
    matchload_state(false);
    //Ramsete path to dual_ball
    //ramsete.followPath(right_1, {.exit_points = 0, .end_correction = false});
    chassis.turnToPoint(-4, -48, 500);
    relativeMotion(-20, -24, 143, 24.5, 1000, true);
    chassis.waitUntilDone();
    matchload_state(true);
    pros::delay(dual_ball_delay);


    //Ramsete backward path to -47, -47

    resting_state();
    chassis.turnToPoint(-24, -47, 850, {.forwards = false});
    distanceReset(true);

    chassis.moveToPoint(-24 + longgoal_offset, -47, 750, {.forwards = false});
    chassis.waitUntilDone();
    resting_state();
    score_longgoal();
    distanceReset(true);
    pros::delay(longgoal_delay);
    resting_state();
    chassis.turnToPoint(-73, -46, 200);
    chassis.moveToPoint(-72 + matchload_offset, -47, 850, {.maxSpeed = 85});
    chassis.waitUntilDone();
    distanceReset(true);
    intake();
    pros::delay(matchload_delay + 300);
    chassis.turnToPoint(-24, -51, 300, {.forwards = false});
    chassis.moveToPoint(-24 - longgoal_offset, -47, 1000, {.forwards = false});
    chassis.waitUntilDone();
    resting_state();
    score_longgoal();
    distanceReset(true);
    pros::delay(longgoal_delay);
    matchload_state(false);
    chassis.moveToPose(-42.25, -30.5, 0, 750);
    chassis.turnToPoint(-17.25, -30.5, 500, {.forwards = false});
    chassis.moveToPoint(-17.25, -30.5, 750, {.forwards = false, .minSpeed = 20});

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
    chassis.setPose(0, 0, 0);
   pros::lcd::print(0, "x: %.2f y: %.2f theta: %.2f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
	chassis.moveToPose(-17, -30, 90, 1250, {.forwards = false, .horizontalDrift = 2, .lead = 0.3});
	chassis.waitUntilDone();
	chassis.moveToPoint(16, -30, 2000, {.forwards = true, .minSpeed = 100});
	chassis.waitUntilDone();
	chassis.moveToPose(27, 0, 0, 3000, {.forwards = true, .horizontalDrift = 2, .lead = 0.5, .minSpeed = 70});
	chassis.waitUntilDone();
	chassis.turnToHeading(0, 2000, {}, true);
   chassis.waitUntilDone();
   pros::delay(200);
   int tube = (chassis.getPose().x + ((frontDistance.get_distance() - 440) / 25.4));
   pros::screen::print(pros::E_TEXT_MEDIUM, 4, "tube pos: %d, dis: %d, current: %2f", tube, frontDistance.get_distance(), chassis.getPose().x);
   pros::delay(200);
   pros::delay(200);
	chassis.moveToPose(tube, 20, 0, 1000, {.forwards = true, .maxSpeed = 80, .minSpeed = 70});
   chassis.waitUntilDone();
   chassis.moveToPoint(chassis.getPose().x, 22, 700, {.forwards = true, .minSpeed = 100});
	for(int i = 0; i<2; i++){
		chassis.moveToPoint(chassis.getPose().x, 21, 700, {.forwards = false, .minSpeed = 70});
		chassis.moveToPoint(chassis.getPose().x, 22, 1000, {.forwards = true, .minSpeed = 100});
	}
   pros::delay(2000);

   chassis.moveToPose(tube, -5, 0, 1000, {.forwards = false});
   chassis.turnToHeading(0, 2000, {}, true);
   chassis.waitUntilDone();
   pros::delay(200);
   int goal = (chassis.getPose().x + ((frontDistance.get_distance() - 395) / 25.4));
   pros::screen::print(pros::E_TEXT_MEDIUM, 6, "goal pos: %d, dis: %d, current: %2f", goal, frontDistance.get_distance(), chassis.getPose().x);
   pros::delay(500);
	chassis.moveToPose(goal, -22, 0, 2000, {.forwards = false, .horizontalDrift = 2, .lead = 0.4});
   chassis.waitUntilDone();
	chassis.moveToPose(12, -2, 270, 7000, {.forwards = true, .horizontalDrift = 2, .lead = 0.6, .minSpeed = 90, .earlyExitRange = 0.5});
	chassis.moveToPose(2, -60, 180, 3000, {.forwards = true, .horizontalDrift = 2, .lead = 0.5});
   chassis.turnToHeading(180, 2000);
   chassis.waitUntilDone();
   chassis.moveToPoint(2, -100, 7000, {.forwards = true});
   chassis.waitUntilDone();
   

   chassis.turnToHeading(180, 2000, {}, true);
   chassis.waitUntilDone();
   pros::delay(500);
}

void awp_auton() {
    /*
    MCL::StartMCL(-51.25, -18.5, 180);
    pros::Task mclTask(MCL::MonteCarlo);
    enable_fused_odometry(true);
    */
    //colorSort(Color::RED);
    //FIRST LONGGOAL
    
    chassis.setPose(-51.25, -18.5, 180);
    chassis.moveToPoint(-51.25, -48, 2000);
    matchload_state(true);
    chassis.waitUntilDone();

    intake();
    chassis.turnToPoint(-72, -48, 2000);
    chassis.waitUntilDone();
    
    chassis.moveToPoint(-72 + matchload_offset, -48, 2000, {.minSpeed=25});
    chassis.waitUntilDone();
    //pros::delay(matchload_delay);
    chassis.turnToPoint(-22 - longgoal_offset, -52, 2000, {.forwards = false});
    chassis.moveToPoint(-22 - longgoal_offset, -52, 2000, {.forwards = false,.minSpeed=20});
    chassis.waitUntilDone();
    resting_state();
    matchload_state(false);
    score_longgoal();
    distancePose pose = distanceReset(true);
    pros::delay(longgoal_delay + 300);
    resting_state();

    //MIDDLE AUTON
    relativeMotion(pose.x, pose.y, 270, 4, 1000, true);
    chassis.turnToPoint(-20, -24, 2000);
    intake();
    chassis.moveToPoint(-20 , -24, 2000, {.maxSpeed = 85});
    chassis.waitUntil(18);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-20, 29, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-20, 26.5, 2000, {.maxSpeed = 85});
    chassis.waitUntil(5);
    matchload_state(false);
    chassis.waitUntil(38.5);
    matchload_state(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-11.5, 12.5, 2000, {.forwards = false});
    chassis.moveToPoint(-11.5, 12.5, 2000, {.forwards = false, .minSpeed=20});
    chassis.waitUntilDone();
    matchload_state(false);
    score_midgoal();
    pros::delay(midgoal_delay);
    
    //2nd LONGGOAL
    chassis.moveToPoint(-47, 47, 2000);
    chassis.waitUntilDone();
    matchload_state(true);
    chassis.turnToPoint(-72, 47, 2000);
    distanceReset(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-72 + matchload_offset, 47, 2000,{.minSpeed=30});

    resting_state(); // reset trapdoor from mid goal here
    intake();
    chassis.waitUntilDone();
    chassis.turnToPoint(-24, 48, 2000, {.forwards = false});
    pros::delay(matchload_delay);

    chassis.moveToPoint(-22 - longgoal_offset, 48, 2000, {.forwards = false});
    chassis.waitUntil(10);
    matchload_state(false);
    score_longgoal();
}

void skills_auton() {

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
    chassis.turnToPoint(-11.5, 11.5, 1000, {.forwards = false});
    chassis.moveToPoint(-11.5, 11.5, 1500, {.forwards = false});
    chassis.waitUntilDone();
    
    chassis.turnToPoint(-47, 50, 1000);
    chassis.moveToPoint(-47, 50, 2000);
    chassis.turnToPoint(-72 + matchload_offset, 51, 1000);
    chassis.waitUntilDone();
    distanceReset(true);
    chassis.moveToPoint(-72 + matchload_offset, 51, 1500);
    chassis.waitUntilDone();
    ramsete.followPath(skills_1, {.backwards = true});
    chassis.moveToPoint(22 + longgoal_offset, 62, 3000, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    distanceReset(true);
    chassis.turnToPoint(47, 48, 1500, {.forwards = false});
    chassis.moveToPoint(47, 48, 2000, {.forwards = false});
    chassis.waitUntilDone();
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