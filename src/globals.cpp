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
#include "pros/optical.hpp"
#include "pros/vision.hpp"
#include "colorSort.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({17, 19, -18}, pros::MotorGears::blue);
pros::MotorGroup leftMotors({-12, 13, -14}, pros::MotorGears::blue);

lemlib::Drivetrain drivebase(
    &leftMotors, 
    &rightMotors, 
    11.55, 
    3.25, 
    450, 
    8);

pros::Imu imu(16);
pros::Rotation horizontal_tracking_sensor(13);
pros::Rotation vertical_tracking_sensor(-12);
//-6.95780365196
//0.0188794350939
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, 2,-7.28684844735 , 1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, 2, 0.0188794350939,1);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

lemlib::ControllerSettings lateral_controller(16.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              120, // derivative gain (kD)
                                              4, // anti windup
                                              0.5, // small error range, in inches
                                              75, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              25 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(3.2, // proportional gain (kP)
                                              0.0, // integral gain (kI)
                                              25, // derivative gain (kD)
                                               3, // anti windup
                                              0.5, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              75, // large error range timeout, in milliseconds
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


Color allianceColor = Color::RED;
bool color_sort_enable = false;
bool midgoal_first = false;
int ramp_up_time = 0;
int low_ramp_down_time = 0;