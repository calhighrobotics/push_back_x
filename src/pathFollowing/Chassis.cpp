#include "Chassis.h"
#include <cmath>
#include <algorithm>
#include <sys/_intsup.h>
#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"

constexpr float INCH_TO_METER = 0.0254f;

Chassis::Chassis(lemlib::Drivetrain drivetrain,
                 lemlib::ControllerSettings lateralSettings,
                 lemlib::ControllerSettings angularSettings,
                 lemlib::OdomSensors sensors,
                 lemlib::DriveCurve* throttleCurve,
                 lemlib::DriveCurve* steerCurve)
    : lemlib::Chassis(drivetrain, lateralSettings, angularSettings, sensors, throttleCurve, steerCurve),
      drivetrain(drivetrain) {}

Chassis::WheelPowers Chassis::desaturate(float lateral, float angular, float maxSpeed) {
    float left = lateral + angular;
    float right = lateral - angular;
    float maxMag = std::max(std::abs(left), std::abs(right));
    if (maxMag > maxSpeed) {
        left = (left / maxMag) * maxSpeed;
        right = (right / maxMag) * maxSpeed;
    }
    return {left, right};
}

void Chassis::moveToPointRamsete(float target_x, float target_y, int timeout_msec, const VelocityControllerConfig &config, MoveToPointParams params, bool async) {
    // 1. ASYNC WRAPPER: Spawns task so main thread can use waitUntil()
    if (async) {
        pros::Task task([=]() { moveToPointRamsete(target_x, target_y, timeout_msec, config, params, false); });
        pros::delay(10); // Give task time to start and set motionRunning
        return;
    }

    this->requestMotionStart(); 
    
    target_x *= INCH_TO_METER;
    target_y *= INCH_TO_METER;
    float earlyExit_m = params.earlyExitRange * INCH_TO_METER;
    
    VoltageController controller(
        config.kV, config.KA_straight, config.KA_turn,
        config.KS_straight, config.KS_turn,
        config.KP_straight, config.KI_straight,
        5000.0, 11.45 * INCH_TO_METER
    );

    lemlib::PID angularPID(11, 0.001, 0);
    lemlib::PID lateralPID(5, 0, 0);
    lemlib::ExitCondition lateral(0.04, 50);
    lemlib::ExitCondition longitudinal(0.01, 50);
    
    int timeElapsed = 0;
    float lateralGain = 0;
    float pastAngularVelocity = 0;
    float pastLinearVelocity = 0;
    
    auto sign = [](float x) { return (x > 0) ? 1.0f : ((x < 0) ? -1.0f : 0.0f); };
    auto sinc = [](float x) { return (std::abs(x) < 1e-5) ? 1.0f : std::sin(x) / x; };

    float rpm_to_mps_factor = 0.00324173f; 
    bool is_settling = false; 

    // Used for tracking chassis.waitUntil() distance
    lemlib::Pose lastPose = this->getPose(true, true);

    while (timeElapsed < timeout_msec && (!lateral.getExit() || !longitudinal.getExit())) {
        if (!this->motionRunning) break; // Allows chassis.cancelMotion() to work
        
        lemlib::Pose currentPoseRaw = this->getPose(true, true);
        
        // 2. DISTANCE TRACKING: Updates LemLib's internal distance tracker for waitUntil()
        this->distTraveled += lastPose.distance(currentPoseRaw);
        lastPose = currentPoseRaw;

        lemlib::Pose currentPose = currentPoseRaw;
        currentPose.x *= INCH_TO_METER;
        currentPose.y *= INCH_TO_METER;
        
        float d = std::hypot(target_x - currentPose.x, target_y - currentPose.y);

        float dx = target_x - currentPose.x;
        float dy = target_y - currentPose.y;

        if (!params.forwards) {
            dx = -dx;
            dy = -dy;
        }

        float theta = currentPose.theta;
        float localErrorX = std::cos(theta) * dx + std::sin(theta) * dy;
        float localErrorY = -std::sin(theta) * dx + std::cos(theta) * dy;

        float angularError = std::atan2(localErrorY, localErrorX);
        float cosineScaling = std::cos(angularError);
        float driveError;

        if (d < 0.05f) {
            is_settling = true;
        }

        if (is_settling) { 
            angularError = 0;     
            cosineScaling = 1.0f; 
            driveError = std::cos(theta) * dx + std::sin(theta) * dy;
        } else {
            driveError = d * sign(cosineScaling); 
        }

        if (params.minSpeed > 0 && (d < earlyExit_m || driveError < 0)) {
            break;
        }

        lateral.update(localErrorY);
        longitudinal.update(localErrorX);

        float angularOutput = angularPID.update(angularError);
        float driveOutput = lateralPID.update(driveError);
        
        driveOutput = std::clamp(driveOutput, -params.maxSpeed, params.maxSpeed);
        driveOutput = driveOutput * std::abs(cosineScaling);

        if (is_settling) { 
            angularOutput = 0; 
        } else {
            angularOutput += driveOutput * lateralGain * localErrorY * sinc(angularError);
        }

        if (params.minSpeed > 0 && std::abs(driveOutput) < params.minSpeed) {
            driveOutput = params.minSpeed * sign(driveOutput);
            if (driveOutput == 0) driveOutput = params.minSpeed;
        }

        angularOutput = std::clamp(angularOutput, -params.maxTurnSpeed, params.maxTurnSpeed);
    
        if (!params.forwards) {
            driveOutput = -driveOutput;
        }

        float leftVel = drivetrain.leftMotors->get_actual_velocity() * rpm_to_mps_factor;
        float rightVel = drivetrain.rightMotors->get_actual_velocity() * rpm_to_mps_factor;
        
        DrivetrainVoltages outputVoltages = controller.update(driveOutput, angularOutput, leftVel, rightVel);
        
        drivetrain.leftMotors->move_voltage(outputVoltages.leftVoltage * 1000);
        drivetrain.rightMotors->move_voltage(outputVoltages.rightVoltage * 1000);
        
        pastAngularVelocity = angularOutput;
        pastLinearVelocity = driveOutput;

        pros::delay(10);
        timeElapsed += 10;
    }

    if (params.minSpeed == 0) {
        drivetrain.leftMotors->move_voltage(0);
        drivetrain.rightMotors->move_voltage(0);
    }
    
    // Signals to waitUntilDone() and LemLib motion chaining that this movement is over
    this->motionRunning = false;
}

void Chassis::moveToPoseRamsete(float targetX, float targetY, float targetTheta, int timeout_msec, const VelocityControllerConfig &config, MoveToPoseParams params, bool async) {
    // 1. ASYNC WRAPPER
    if (async) {
        pros::Task task([=]() { moveToPoseRamsete(targetX, targetY, targetTheta, timeout_msec, config, params, false); });
        pros::delay(10); 
        return;
    }

    this->requestMotionStart(); 
    
    targetX *= INCH_TO_METER;
    targetY *= INCH_TO_METER;
    float settleRadius_m = params.settleRadius * INCH_TO_METER;
    float earlyExit_m = params.earlyExitRange * INCH_TO_METER; 

    VoltageController controller(
        config.kV, config.KA_straight, config.KA_turn,
        config.KS_straight, config.KS_turn,
        config.KP_straight, config.KI_straight,
        5000.0, 11.45 * INCH_TO_METER
    );

    lemlib::PID angularPID(7.5, 0.001, 26.5);
    lemlib::PID lateralPID(5.5 , 0.001 , 27);
    
    lemlib::ExitCondition lateralExit(0.04, 50);
    lemlib::ExitCondition longitudinalExit(0.025, 50);

    float pastAngularVelocity = 0;
    float pastLinearVelocity = 0;
    int timeElapsed = 0;

    auto sign_func = [](float x) { return (x > 0) ? 1.0f : ((x < 0) ? -1.0f : 0.0f); };
    auto sinc = [](float x) { return (std::abs(x) < 1e-5) ? 1.0f : std::sin(x) / x; };

    float rpm_to_mps_factor = 0.00324173f; 

    float targetThetaRad = targetTheta * (M_PI / 180.0f);
    float end_unit_x = std::sin(targetThetaRad);
    float end_unit_y = std::cos(targetThetaRad);
    float dir_sign = params.forwards ? 1.0f : -1.0f;

    bool is_settling = false;

    // Distance tracking for waitUntil()
    lemlib::Pose lastPose = this->getPose(true, true);

    while (timeElapsed < timeout_msec && (!lateralExit.getExit() || !longitudinalExit.getExit())) {
        if (!this->motionRunning) break; 
        
        lemlib::Pose currentPoseRaw = this->getPose(true, true);
        
        // 2. DISTANCE TRACKING 
        this->distTraveled += lastPose.distance(currentPoseRaw);
        lastPose = currentPoseRaw;

        lemlib::Pose currentPose = currentPoseRaw;
        currentPose.x *= INCH_TO_METER;
        currentPose.y *= INCH_TO_METER;
        float theta = currentPose.theta;

        float d = std::hypot(targetX - currentPose.x, targetY - currentPose.y);

        float true_dx = targetX - currentPose.x;
        float true_dy = targetY - currentPose.y;
        if (!params.forwards) {
            true_dx = -true_dx;
            true_dy = -true_dy;
        }
        float true_localErrorX = std::cos(theta) * true_dx + std::sin(theta) * true_dy;
        float true_localErrorY = -std::sin(theta) * true_dx + std::cos(theta) * true_dy;

        float rx = currentPose.x - targetX;
        float ry = currentPose.y - targetY;
        float along_track = rx * end_unit_x + ry * end_unit_y;
        float dynamic_lookahead = d * params.lead;
        float lookahead_m = std::max(dynamic_lookahead, 0.35f);
        float ghost_along_track = along_track + (lookahead_m * dir_sign);
        
        float carrot_x = targetX + ghost_along_track * end_unit_x;
        float carrot_y = targetY + ghost_along_track * end_unit_y;

        float dx = carrot_x - currentPose.x;
        float dy = carrot_y - currentPose.y;
        if (!params.forwards) {
            dx = -dx;
            dy = -dy;
        }

        float localErrorX = std::cos(theta) * dx + std::sin(theta) * dy;
        float localErrorY = -std::sin(theta) * dx + std::cos(theta) * dy;

        float heading_error_rad = std::atan2(localErrorY, localErrorX);
        float cosine_scale = std::cos(heading_error_rad);

        float error_norm_sq = localErrorX * localErrorX + localErrorY * localErrorY;
        float driveError = std::sqrt((error_norm_sq + 2.0f * d * d) / 3.0f) * sign_func(cosine_scale);
        float turnError = heading_error_rad;

        if (d < std::max(settleRadius_m, 0.05f)) {
            is_settling = true;
        }

        if (is_settling) {
            float targetThetaMath = M_PI_2 - targetThetaRad;
            float final_error = targetThetaMath - theta;
            final_error = std::remainder(final_error, 2.0 * M_PI);
            
            turnError = final_error;
            driveError = true_localErrorX; 
            cosine_scale = 1.0f; 
        }

        if (params.earlyExitRange > 0 && (d < earlyExit_m || driveError < 0)) {
            break; 
        }

        lateralExit.update(true_localErrorY);
        longitudinalExit.update(true_localErrorX); 

        float driveOutput = lateralPID.update(driveError);
        float turnOutput = angularPID.update(turnError);

        driveOutput = std::clamp(driveOutput, -params.maxSpeed, params.maxSpeed);

        if (!is_settling) {
            float steeringVelocity = std::max(std::abs(driveOutput), 0.2f);
            turnOutput += steeringVelocity * params.k_lat * localErrorY * sinc(heading_error_rad);
        }

        driveOutput = std::abs(cosine_scale) * driveOutput;
        
        if (!is_settling && params.minSpeed > 0 && std::abs(driveOutput) < params.minSpeed) {
            driveOutput = params.minSpeed * sign_func(driveOutput);
            if (driveOutput == 0) driveOutput = params.minSpeed;
        }

        if (!params.forwards) {
            driveOutput = -driveOutput; 
        }

        turnOutput = std::clamp(turnOutput, -params.maxTurnSpeed, params.maxTurnSpeed);

        float leftVel = drivetrain.leftMotors->get_actual_velocity() * rpm_to_mps_factor;
        float rightVel = drivetrain.rightMotors->get_actual_velocity() * rpm_to_mps_factor;
        
        DrivetrainVoltages outputVoltages = controller.update(driveOutput, turnOutput, leftVel, rightVel);
        
        drivetrain.leftMotors->move_voltage(outputVoltages.leftVoltage * 1000);
        drivetrain.rightMotors->move_voltage(outputVoltages.rightVoltage * 1000);
        
        pastAngularVelocity = turnOutput;
        pastLinearVelocity = driveOutput;

        pros::delay(10);
        timeElapsed += 10;
    }

    if (params.minSpeed == 0) {
        drivetrain.leftMotors->move_voltage(0);
        drivetrain.rightMotors->move_voltage(0);
    }
    
    this->motionRunning = false;
}

void Chassis::tank(float lin_vel, float ang_vel, const VelocityControllerConfig &config, unsigned int time)
{
    VoltageController controller(
        config.kV, config.KA_straight, config.KA_turn,
        config.KS_straight, config.KS_turn,
        config.KP_straight, config.KI_straight,
        5000.0, 11.45 * INCH_TO_METER
    );

    u_int32_t starting_time = pros::millis();
    
    while(pros::millis() - starting_time < time)
    {
        float leftVel = drivetrain.leftMotors->get_actual_velocity() * 0.00324173f;
        float rightVel = drivetrain.rightMotors->get_actual_velocity() * 0.00324173f;
        
        DrivetrainVoltages outputVoltages = controller.update(lin_vel, ang_vel, leftVel, rightVel);
    
        drivetrain.leftMotors->move_voltage(outputVoltages.leftVoltage * 1000);
        drivetrain.rightMotors->move_voltage(outputVoltages.rightVoltage * 1000);
    
        pros::delay(10);
    }
    
    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->brake();
}

void Chassis::brake(pros::MotorBrake brake_mode)
{
    drivetrain.rightMotors->set_brake_mode(brake_mode);
    drivetrain.leftMotors->set_brake_mode(brake_mode);
    drivetrain.rightMotors->brake();
    drivetrain.leftMotors->brake();
}

void Chassis::tank(int left, int right)
{
    drivetrain.rightMotors->move(right);
    drivetrain.leftMotors->move(left);
}

bool Chassis::detect_collision()
{
    return true;
}