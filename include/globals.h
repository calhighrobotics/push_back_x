#pragma once

#include "pros/distance.hpp"
#include "pros/misc.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/vision.hpp"


struct State {
    float x, y, heading, linear_vel, angular_vel;
} ;




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



extern pros::Distance rightDistance;
extern pros::Distance leftDistance;
extern pros::Distance frontDistance;
extern pros::Distance backDistance;


extern pros::adi::Pneumatics trapDoor;
extern pros::adi::Pneumatics matchload;
extern pros::adi::Pneumatics basket;

extern pros::Optical color_sensor;

extern pros::Vision vision_sensor;

