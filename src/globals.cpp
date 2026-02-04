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
#include "colorSort.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({-8,10,9}, pros::MotorGears::blue);
pros::MotorGroup leftMotors({1,-3,-2,}, pros::MotorGears::blue);

lemlib::Drivetrain drivebase(
    &leftMotors, 
    &rightMotors, 
    11.55, 
    lemlib::Omniwheel::NEW_325, 
    450, 
    5);

pros::Imu imu(5);

pros::Rotation horizontal_tracking_sensor(-13);
pros::Rotation vertical_tracking_sensor(-12);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, lemlib::Omniwheel::NEW_2, -6.26707246263, 1); //Units are in inches
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, lemlib::Omniwheel::NEW_2, -0.091865514797,1);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              3, // anti windup
                                              0.5, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              150, // large error range timeout, in milliseconds
                                              35 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0.0015, // integral gain (kI)
                                              24, // derivative gain (kD)
                                               3, // anti windup
                                              1, // small error range, in inches
                                              25, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              50, // large error range timeout, in milliseconds
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
pros::Motor topMotor(19, pros::v5::MotorGears::blue);


pros::Distance rightDistance(20);
pros::Distance leftDistance(15);
pros::Distance frontDistance(14);
pros::Distance backDistance(16);

pros::adi::Pneumatics trapDoor('A', false);
pros::adi::Pneumatics matchload('E', false);
pros::adi::Pneumatics basket('F', false);
pros::adi::Pneumatics descore('D', false);
pros::adi::Pneumatics lowGoalAligner('B', true);
pros::adi::Pneumatics intakeFunnel('C', false);

pros::Optical color_sensor(7);

pros::Vision vision_sensor(16);

Color allianceColor = Color::RED;
bool color_sort_enable = true;
bool midgoal_first = false;
int ramp_up_time = 0;
int low_ramp_down_time = 0;