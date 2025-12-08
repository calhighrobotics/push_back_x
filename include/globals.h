#pragma once

#include "pros/distance.hpp"
#include "pros/misc.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

extern const float INCH_TO_METER;

struct State {
    float x, y, heading, linear_vel, angular_vel;
} ;

class Vector2 {
    public:
        Vector2(float x, float y);

        std::string latex() const;

        float x;
        float y;
};

extern pros::Controller controller;

extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;

extern lemlib::Drivetrain drivebase;

extern pros::IMU imu;
extern pros::Rotation horizontal_tracking_sensor;
extern pros::Rotation vertical_tracking_sensor;

extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

extern lemlib::Chassis chassis;
extern pros::Motor intakeMotor;
extern pros::Motor topMotor;



extern pros::Distance right;
extern pros::Distance left;
extern pros::Distance front;
extern pros::Distance back;


extern pros::adi::Pneumatics A;
extern pros::adi::Pneumatics B;
extern pros::adi::Pneumatics C;


