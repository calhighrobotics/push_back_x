#include "main.h"
#include "globals.h" 
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/core/lv_obj_pos.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/vision.hpp"
#include "robodash/views/selector.hpp"
#include <string>
#include "MCL.h"
#include "distanceReset.h"
#include <atomic> 
#include <memory> 

rd::Console console;
void initialize() {
    chassis.calibrate();
    console.focus();
    double start_x = -44.77;
    double start_y = -12.31;
    double start_theta = 90;
    chassis.setPose(start_x, start_y, start_theta); 
    MCL::StartMCL(start_x, start_y);
    pros::Task mcl_task(MCL::MonteCarlo);
    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
            
            lemlib::Pose odomPose = chassis.getPose();
            
            console.printf("[ RAW ODOMETRY ]\n");
            console.printf("X: %.2f  Y: %.2f\n", odomPose.x, odomPose.y);
            console.printf("Theta: %.2f\n\n", odomPose.theta);
            
            console.printf("[ MCL ESTIMATE ]\n");
            console.printf("X: %.2f  Y: %.2f\n", MCL::global_X, MCL::global_Y);

			//distancePose pose = distanceReset(false);
			//console.printf("X DSR: %.2f  Y DSR : %.2f\n", pose.x, pose.y);

            
            pros::delay(50);
        }
    });
}

void disabled() {
    
}

void competition_initialize() {
}

extern lemlib::Chassis chassis;

struct Reading {
    double heading;
    double distance;
};

void driveArcadeVoltage(float leftVoltage, float rightVoltage) {
    float max = std::max(fabs(leftVoltage), fabs(rightVoltage));
    if (max > 12.0) {
        leftVoltage *= 12.0 / max;
        rightVoltage *= 12.0 / max;
    }
    leftMotors.move_voltage(leftVoltage); 
    rightMotors.move_voltage(rightVoltage);
}

DrivetoPointConfig dtpConfig{};

// Wraps any angle (in degrees) to the range [-90, 90]
float wrapTo90(float angle) {
    while (angle > 90.0f) angle -= 180.0f;
    while (angle < -90.0f) angle += 180.0f;
    return angle;
}

void driveToPoint(float target_x, float target_y, const VelocityControllerConfig &config) {
    // Set up position variables
    target_x *= INCH_TO_METER;
    target_y *= INCH_TO_METER;
    lemlib::Pose currentPose = chassis.getPose(true, true);
    lemlib::Pose target(target_x, target_y);

    VoltageController controller(
        config.kV,
        config.KA_straight,
        config.KA_turn,
        config.KS_straight,
        config.KS_turn,
        config.KP_straight,
        config.KI_straight,
        99999.0,
        10.0 * INCH_TO_METER
    );
    // working angular was kp 2, kd, 10, lateralGain 2
    lemlib::PID angularPID(2, 0, 1);
    lemlib::PID lateralPID(1.5, 0.0005, 0);
    lemlib::ExitCondition lateral(0.04, 50);
    lemlib::ExitCondition longitudinal(0.01, 50);
    float lateralGain = 1.75;
    float pastAngularVelocity = 0;
    float pastLinearVelocity = 0;
    // Logging variables
    float time = 0.0;
    std::vector<std::string> logs;

    while (!lateral.getExit() || !longitudinal.getExit()) {
        currentPose = chassis.getPose(true, true);
        currentPose.x *= INCH_TO_METER;
        currentPose.y *= INCH_TO_METER;
        Eigen::Vector2f globalError(target_x-currentPose.x, target_y-currentPose.y);
        Eigen::Matrix2f rotationMatrix;
        rotationMatrix << cosf(currentPose.theta), sinf(currentPose.theta),
                -sinf(currentPose.theta), cosf(currentPose.theta);
        Eigen::Vector2f localError = rotationMatrix * globalError;

        float angularError = atan2f(localError.y(), localError.x());
        float cosineScaling = cosf(angularError);
        float driveError = localError.norm() * (float) sign(cosineScaling);

        // Set angular error equal to 0 when close to the target; following point is no longer important
        if (fabsf(driveError) < 0.07) { // in meters
            angularError = 0;
            cosineScaling = (float) sign(cosineScaling);
        }

        // Update exit conditions
        lateral.update(localError.y());
        longitudinal.update(localError.x());

        // Update angularOutput and clamp it to a respectable value (8.4 rad/s, around max turning speed of robot)
        float angularOutput = angularPID.update(angularError);

        // Update driveOutput and clamp it to a respectable value (1.2 m/s, around max speed of robot)
        float driveOutput = lateralPID.update(driveError);
        driveOutput = std::clamp(driveOutput, -1.0, 1.0);
        driveOutput = driveOutput * fabsf(cosineScaling);

        // When distance to target is close, completely disable angular outputfloat target
        if (fabsf(driveError) < 0.07) 
            angularOutput = 0;
        else
            angularOutput += driveOutput * lateralGain * localError.y() * (float) sinc(angularError);
        angularOutput = clamp(angularOutput, -8.0, 8.0);
        angularOutput = lemlib::slew(angularOutput, pastAngularVelocity, 3.0f);
        driveOutput = lemlib::slew(driveOutput, pastLinearVelocity, 0.25f);

        //std::cout << Vector2(time, angularOutput).latex() << ",";
        DrivetrainVoltages outputVoltages = controller.update(driveOutput, angularOutput, leftMotors.get_actual_velocity()*rpm_to_mps_factor, rightMotors.get_actual_velocity()*rpm_to_mps_factor);
        leftMotors.move_voltage(outputVoltages.leftVoltage*1000);
        rightMotors.move_voltage(outputVoltages.rightVoltage*1000);
        pastAngularVelocity = angularOutput;
        pastLinearVelocity = driveOutput;

        std::cout << Vector2(time, localError.y()*39.37).latex() << ",";
        //std::cout << Vector2(time, angularError).latex() << ",";
        //std::cout << Vector2(time, localError.y()).latex() << ",";
        pros::delay(10);
        time += 0.01;
    }
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);

void autonomous() {
    
    chassis.setPose(-44.77, -12.31, 90);
	chassis.moveToPoint(-24, -12.31, 2000);
	chassis.turnToPoint(-24, 24, 2000);
	chassis.moveToPoint(-24, 24, 2000);
	chassis.moveToPose(24, 24, 90, 10000);
	chassis.moveToPose(24, -24, 180, 10000);
    chassis.turnToPoint(-40, -24, 2000);
    chassis.moveToPoint(-40, -24, 2000);
    chassis.turnToPoint(-40, 60, 2000);
    chassis.moveToPoint(-40, 59, 5000);
    chassis.turnToPoint(40, 60, 2000);
    chassis.moveToPoint(40, 60, 10000);
    chassis.turnToPoint(40, -59, 10000);
    chassis.moveToPoint(40, -60, 10000);
    chassis.turnToPoint(-40, -60, 10000);
    chassis.moveToPoint(-40, -60, 10000);
    chassis.turnToPoint(-40, 0, 5000);
    chassis.moveToPoint(-40, 0 , 5000);
	
    
}
void opcontrol()
{

    while (true) {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    
        chassis.curvature(throttle, steer, false);

        pros::delay(10);
    }
}