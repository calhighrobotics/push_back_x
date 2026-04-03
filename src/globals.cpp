
#include "lemlib/logger/baseSink.hpp"
#include "pros/distance.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/vision.hpp"
#include "colorSort.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({17, 19, -18}, pros::MotorGears::blue);
pros::MotorGroup leftMotors({-12, 13, -14}, pros::MotorGears::blue);

lemlib::Drivetrain drivebase(
    &leftMotors, 
    &rightMotors, 
    11, 
    3.25, 
    450, 
    8);

pros::Imu imu(16);
pros::Rotation horizontal_tracking_sensor(-10);
pros::Rotation vertical_tracking_sensor(-9);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, 2,-2.4373261421 , 1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, 2, -0.273535108698,1);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);


float lateralKP = 5, lateralKI = 0, lateralKD = 0, angularKP = 2, angularKI = 0, angularKD = 0;


lemlib::ControllerSettings lateral_controller(lateralKP, // proportional gain (kP)
                                              lateralKI, // integral gain (kI)
                                              lateralKD, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(angularKP, // proportional gain (kP)
                                              angularKI, // integral gain (kI)
                                              angularKD, // derivative gain (kD)
                                               0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(10,    // joystick deadband out of 127
                                            20,   // minimum output where drivetrain will move out of 127
                                            1.012 // expo curve gain
);

lemlib::ExpoDriveCurve steer_curve(10,   // joystick deadband out of 127
                                        20,   // minimum output where drivetrain will move out of 127
                                         1.017 // expo curve gain
);




lemlib::Chassis chassis(drivebase, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

pros::Motor intakeMotor(-1, pros::v5::MotorGears::blue);
pros::Motor storageMotor(2, pros::v5::MotorGears::blue);
pros::Motor outtakeMotor(3, pros::MotorGears::blue);

pros::Distance rightDistance(20);
pros::Distance leftDistance(15);
pros::Distance frontDistance(14);
pros::Distance backDistance(19);
pros::Distance frontDistance2(17);

pros::adi::Pneumatics intake_lift('A', false);
pros::adi::Pneumatics hood('B', false);
pros::adi::Pneumatics matchloader('C', false);
pros::adi::Pneumatics descore('D', false);

pros::Optical color_sensor(7);

pros::Vision vision_sensor(18);

Color allianceColor = Color::RED;
bool color_sort_enable = false;
bool midgoal_first = false;
int ramp_up_time = 0;
int low_ramp_down_time = 0;

