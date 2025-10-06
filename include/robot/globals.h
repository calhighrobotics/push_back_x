#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.hpp"

extern pros::Controller controller;

extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;

extern lemlib::Drivetrain drivetrain;

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