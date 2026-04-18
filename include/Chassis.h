#pragma once

#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "velocityController.h" 
#include <sys/_types.h>

struct MoveToPointParams {
    bool forwards = true;
    float maxSpeed = 1.4f;   
    float maxTurnSpeed = 10.0f;   
    float minSpeed = 0.0f;      
    float earlyExitRange = 0.0f;
};

struct MoveToPoseParams {
    bool forwards = true;
    float maxSpeed = 1.6f;       
    float maxTurnSpeed = 10.0f;   
    float minSpeed = 0.0f;      
    float lead = 0.6;
    float settleRadius = 4.0f;   
    float k_lat = 27.5;  
    float earlyExitRange = 0.0f; 
};

class Chassis : public lemlib::Chassis {
public:
    lemlib::Drivetrain drivetrain;

    Chassis(lemlib::Drivetrain drivetrain,
            lemlib::ControllerSettings lateralSettings,
            lemlib::ControllerSettings angularSettings,
            lemlib::OdomSensors sensors,
            lemlib::DriveCurve* throttleCurve = nullptr,
            lemlib::DriveCurve* steerCurve = nullptr);

    struct WheelPowers { float left; float right; };
    WheelPowers desaturate(float lateral, float angular, float maxSpeed);

    // Note the bool async = false at the end
    void moveToPointRamsete(float target_x, float target_y, int timeout_msec, const VelocityControllerConfig &config, MoveToPointParams params = {}, bool async = true);

    void moveToPoseRamsete(float targetX, float targetY, float targetTheta, int timeout_msec, const VelocityControllerConfig &config, MoveToPoseParams params = {}, bool async = true);

    void tank(int left, int right);
    void tank(float linear_velocity, float angular_velocity, const VelocityControllerConfig &config, unsigned int time);

    void brake(pros::MotorBrake brake_mode = pros::MotorBrake::coast);
    
    bool collision_montitoring(unsigned int time_msec = -1);
    
    bool detect_collision();
};