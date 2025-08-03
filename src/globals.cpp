#include "pros/misc.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({8,-9,10});
pros::MotorGroup leftMotors({-1,2,-3});

lemlib::Drivetrain drivebase(
    &leftMotors, 
    &rightMotors, 
    11.5, 
    lemlib::Omniwheel::NEW_325, 
    450, 
    2);

pros::IMU imu(4);

pros::Rotation horizontal_tracking_sensor(14);
pros::Rotation vertical_tracking_sensor(7);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, 2, 6.5, 1); //Units are in inches
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, 2, -0.1,1);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(30, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              24.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(5,    // joystick deadband out of 127
                                      35,   // minimum output where drivetrain will move out of 127
                                      1.002 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(5, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.012 // expo curve gain
);

lemlib::Chassis chassis(drivebase, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

pros::Motor intakeMotor(6);
