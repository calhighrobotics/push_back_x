#pragma once
#include "IntakeAntiJam.h"
#include "pros/distance.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/vision.hpp"
#include "colorSort.h"

struct State {
    float x, y, heading, linear_vel, angular_vel;
};

// Controller
extern pros::Controller controller;

// Motor groups
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;

// Drivetrain
extern lemlib::Drivetrain drivebase;

// Sensors
extern pros::IMU imu;
extern pros::Rotation horizontal_tracking_sensor;
extern pros::Rotation vertical_tracking_sensor;

extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;

extern lemlib::OdomSensors sensors;

// PID controllers
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

// Drive curves
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

// Chassis
extern lemlib::Chassis chassis;

// Individual motors
extern pros::Motor intakeMotor;
extern pros::Motor outtakeMotor;
extern pros::Motor storageMotor;

// Distance sensors
extern pros::Distance rightDistance;
extern pros::Distance leftDistance;
extern pros::Distance frontDistance;
extern pros::Distance backDistance;
extern pros::Distance frontDistance2;

// Pneumatics
extern pros::adi::Pneumatics intake_lift;
extern pros::adi::Pneumatics hood;
extern pros::adi::Pneumatics matchloader;
extern pros::adi::Pneumatics descore;
extern pros::adi::Pneumatics mid_descore;

// Optical & vision sensors
extern pros::Optical color_sensor;
extern pros::Vision vision_sensor;

// Robot state variables
extern Color allianceColor;
extern bool color_sort_enable;
extern bool midgoal_first;
extern int ramp_up_time;
extern int low_ramp_down_time;
extern bool skills;
extern bool antiJamEnabled;

extern IntakeAntiJam jamManager;