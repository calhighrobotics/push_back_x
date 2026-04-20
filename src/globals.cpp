
#include "globals.h"
#include "pros/distance.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/vision.hpp"
#include "colorSort.h"
#include "IntakeAntiJam.h"
#include "Chassis.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({17, 19, -18}, pros::MotorGears::blue);
pros::MotorGroup leftMotors({-12, 13, -14}, pros::MotorGears::blue);

lemlib::Drivetrain drivebase(
    &leftMotors, 
    &rightMotors, 
    11.45, 
    3.25, 
    450, 
    8);

//-0.0756927599614
//6.44304298828

//0.066057675731
//6.76806367608

const float horizontal_offset = (-1.18588887482  -1.2351177609)/2;
const float vertical_offset = (-0.414862637269 + 0.0661399466844)/2;

pros::Imu imu(16);
pros::Rotation horizontal_tracking_sensor(-15);
pros::Rotation vertical_tracking_sensor(9);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, 2, horizontal_offset, 1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, 2, 0,1);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);


const float lateralKP = 9, lateralKI = 0.001, lateralKD = 72, angularKP = 3.07, angularKI = 0.0015, angularKD = 26.5;


lemlib::ControllerSettings lateral_controller(lateralKP, // proportional gain (kP)
                                              lateralKI, // integral gain (kI)
                                              lateralKD, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(angularKP, // proportional gain (kP)
                                              angularKI, // integral gain (kI)
                                              angularKD, // derivative gain (kD)
                                               3, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              100, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(10,    // joystick deadband out of 127
                                            20,   // minimum output where drivetrain will move out of 127
                                            1.012 // expo curve gain
);

lemlib::ExpoDriveCurve steer_curve(10,   // joystick deadband out of 127
                                        20,   // minimum output where drivetrain will move out of 127
                                         1 // expo curve gain
);


Chassis chassis(drivebase, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

pros::Motor intakeMotor(-1, pros::v5::MotorGears::blue);
pros::Motor storageMotor(2, pros::v5::MotorGears::blue);
pros::Motor outtakeMotor(3, pros::MotorGears::blue);

pros::Distance rightDistance(6);
pros::Distance leftDistance(5);
pros::Distance frontDistance(8);
pros::Distance backDistance(7);

pros::adi::Pneumatics intake_lift('A', false);
pros::adi::Pneumatics hood('B', false);
pros::adi::Pneumatics matchloader('C', false);
pros::adi::Pneumatics descore('D', true);
pros::adi::Pneumatics mid_descore('E', false);

pros::Optical color_sensor(10);

pros::Vision vision_sensor(18);

Color allianceColor = Color::RED;
bool color_sort_enable = false;
bool midgoal_first = false;
int ramp_up_time = 0;
int low_ramp_down_time = 0;
bool antiJamEnabled = false;
bool liveReplay = false;

IntakeAntiJam jamManager(intakeMotor, outtakeMotor, storageMotor, 55);

