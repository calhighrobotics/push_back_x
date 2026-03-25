#include <sstream>
#include <string>
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/ai_vision.h"
#include "pros/ai_vision.hpp"
#include "pros/colors.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/optical.hpp"
#include "pros/vision.hpp"


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({17, 16, -10}, pros::MotorGears::green);
pros::MotorGroup leftMotors({-14, -15, 6}, pros::MotorGears::green);


lemlib::Drivetrain drivebase(
    &leftMotors, 
    &rightMotors, 
    10.1, 
    4.06, 
    160, 
    5);

pros::Imu imu(11);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ControllerSettings lateral_controller(100, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              200, // derivative gain (kD)
                                              4, // anti windup
                                              0.2, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0.0, // integral gain (kI)
                                              14, // derivative gain (kD)
                                               4, // anti windup
                                              0.3, // small error range, in inches
                                              200, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(20,    // joystick deadband out of 127
                                            10,   // minimum output where drivetrain will move out of 127
                                            1.01 // expo curve gain
);

lemlib::ExpoDriveCurve steer_curve(20,   // joystick deadband out of 127
                                        10,   // minimum output where drivetrain will move out of 127
                                         1.015 // expo curve gain
);


lemlib::Chassis chassis(drivebase, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

pros::Distance rightDistance(4);
pros::Distance leftDistance(3);
pros::Distance frontDistance(1);
pros::Distance backDistance(5);
pros::Distance frontDistance2(2);


