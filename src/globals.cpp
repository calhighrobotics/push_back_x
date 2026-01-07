#include <sstream>
#include <string>
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/colors.hpp"
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
};

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({8,-9,10}, pros::MotorGears::blue);
pros::MotorGroup leftMotors({-1,2,-3}, pros::MotorGears::blue);

lemlib::Drivetrain drivebase(
    &leftMotors, 
    &rightMotors, 
    11.5, 
    lemlib::Omniwheel::NEW_325, 
    450, 
    5);

pros::IMU imu(19);

pros::Rotation horizontal_tracking_sensor(20);
pros::Rotation vertical_tracking_sensor(-16);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, lemlib::Omniwheel::NEW_2, -6.37728606611, 1); //Units are in inches
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, lemlib::Omniwheel::NEW_2, 0.110912828986,1);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              3, // anti windup
                                              0.2, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              15 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0.0015, // integral gain (kI)
                                              24, // derivative gain (kD)
                                               3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(5,    // joystick deadband out of 127
                                            10,   // minimum output where drivetrain will move out of 127
                                            1.01 // expo curve gain
);

lemlib::ExpoDriveCurve steer_curve(5,   // joystick deadband out of 127
                                         10,   // minimum output where drivetrain will move out of 127
                                         1.02 // expo curve gain
);


lemlib::Chassis chassis(drivebase, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

pros::Motor intakeMotor(18, pros::v5::MotorGears::blue);
pros::Motor topMotor(7, pros::v5::MotorGears::blue);


pros::Distance rightDistance(6);
pros::Distance leftDistance(12);
pros::Distance frontDistance(13);
pros::Distance backDistance(11);

pros::adi::Pneumatics trapDoor('A', false);
pros::adi::Pneumatics matchload('B', false);
pros::adi::Pneumatics basket('C', false);
pros::adi::Pneumatics descore('D', true);

pros::Optical color_sensor(14);

pros::Vision vision_sensor(16);






