#pragma once

#include "api.h"
#include "lemlib/api.hpp"
#include "pros/ai_vision.hpp"
#include "pros/motors.h"




// Controller
extern pros::Controller controller;

// Motor ports
extern signed char LEFT_BACK;
extern signed char LEFT_MID;
extern signed char LEFT_FRONT;

extern signed char RIGHT_BACK;
extern signed char RIGHT_MID;
extern signed char RIGHT_FRONT;

extern pros::Motor left_back;
extern pros::Motor left_mid;
extern pros::Motor left_front;

extern pros::Motor right_back;
extern pros::Motor right_mid;
extern pros::Motor right_front;

// Motors
extern pros::Motor flywheel_motor;
extern pros::Motor intake_motor;

extern pros::Motor flywheel_motor;
extern pros::Motor hood_motor;

// Motor groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

//pneumatics
extern pros::adi::DigitalOut chicken_wing;
extern pros::adi::DigitalOut mloader;
extern pros::adi::DigitalOut odom_lifter;
extern pros::adi::DigitalOut indexer;
extern pros::adi::DigitalOut extender;

// Drivetrain
extern lemlib::Drivetrain drivetrain;

// Odometry / Sensors
extern pros::Rotation rotation_horiz;
extern pros::Rotation rotation_vert;
extern pros::Imu imu;
extern lemlib::OdomSensors sensors;

extern pros::Distance front_sensor;
extern pros::Distance left_sensor;
extern pros::Distance right_sensor;
extern pros::Distance back_sensor;

// PID Controllers
extern lemlib::ControllerSettings lateralPID;
extern lemlib::ControllerSettings angularPID;

// Drive curve
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;

// Chassis
extern lemlib::Chassis chassis;

extern pros::AIVision ai_vision;


