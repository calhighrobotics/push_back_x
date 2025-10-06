#pragma once

#include "pros/distance.hpp"
#include "pros/misc.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

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
extern pros::Motor agitator;
extern pros::Motor midMotor;


extern pros::Distance right(4);
extern pros::Distance left(5);
extern pros::Distance front(7);
extern pros::Distance back(8);



extern pros::adi::Pneumatics topRoller;
extern pros::adi::Pneumatics midRollerHeight;
extern pros::adi::Pneumatics matchload_mech;
extern pros::adi::Pneumatics aligner;

