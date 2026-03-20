#include "api.h"
#include "lemlib/api.hpp"
#include "pros/motors.h"




//Initialize the controller variable with the primary controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


signed char LEFT_BACK = -1;
signed char LEFT_MID = 2;
signed char LEFT_FRONT = -7;


signed char RIGHT_BACK = 10;
signed char RIGHT_MID = -9;
signed char RIGHT_FRONT = 8;

signed char intake_port = 5;
signed char hood_port = 4;

pros::Motor intake_motor(intake_port);
pros::Motor hood_motor(hood_port);


pros::Motor left_back (LEFT_BACK, pros::v5::MotorGears::blue);
pros::Motor left_mid (LEFT_MID, pros::v5::MotorGears::blue);
pros::Motor left_front (LEFT_FRONT, pros::v5::MotorGears::blue);

pros::Motor right_back (RIGHT_BACK, pros::v5::MotorGears::blue);
pros::Motor right_mid (RIGHT_MID, pros::v5::MotorGears::blue);
pros::Motor right_front (RIGHT_FRONT, pros::v5::MotorGears::blue);


//Initialize the motor group for the left motors with ports 1, 2, and 3, denoting the blue gear cartrige
pros::MotorGroup leftMotors({LEFT_BACK,LEFT_MID, LEFT_FRONT}, pros::v5::MotorGears::blue);

//Initialize the motor group for the right motors with ports 4, 5, and 6, denoting the blue cartidge
//The negative sign for the ports indicate the motors will run in reverse, as one side always must
pros::MotorGroup rightMotors({RIGHT_BACK, RIGHT_MID, RIGHT_FRONT}, pros::v5::MotorGears::blue);

//Rotation sensor ports
uint8_t rot_sensor_horiz = -19;
uint8_t rot_sensor_vert = 20;

//Horizontal sensor
pros::Rotation rotation_horiz(rot_sensor_horiz);
lemlib::TrackingWheel horizontalTracking(&rotation_horiz, 2.0f, 0.68);

//Vertical sensor
pros::Rotation rotation_vert(rot_sensor_vert);
lemlib::TrackingWheel verticalTracking(&rotation_vert, 2.0f, 0.95);


#define FRONT_DISTANCE_PORT 12
pros::Distance front_sensor(FRONT_DISTANCE_PORT);

#define LEFT_DISTANCE_PORT 18
pros::Distance left_sensor(LEFT_DISTANCE_PORT);

#define RIGHT_DISTANCE_PORT 16
pros::Distance right_sensor(RIGHT_DISTANCE_PORT);

#define BACK_DISTANCE_PORT 17
pros::Distance back_sensor(BACK_DISTANCE_PORT);

#define MATCH_LOADER_PORT 'B'
#define ODOM_LIFTER_PORT 'C'
#define WING_PORT 'A'
#define INDEXER_PORT 'D'
#define EXTENDER_PORT 'E'

pros::adi::DigitalOut mloader (MATCH_LOADER_PORT);
pros::adi::DigitalOut odom_lifter (ODOM_LIFTER_PORT);
pros::adi::DigitalOut chicken_wing (WING_PORT);
pros::adi::DigitalOut indexer (INDEXER_PORT);
pros::adi::DigitalOut extender (EXTENDER_PORT);



lemlib::Drivetrain drivetrain(
    &leftMotors, // the left motor group
    &rightMotors, // the right motor group
    12.75, //25 inch track width
    3.25, //Wheel size
    450, // our drivetrain RPM
    2 // optimal drift value for all-omni drivetrain
);

lemlib::OdomSensors sensors(&verticalTracking, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, DNE
                            &horizontalTracking, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, DNE
                            nullptr // inertial sensor
);

// Lateral (forward/backward)
lemlib::ControllerSettings lateralPID(      15, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            120, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// Angular (turning)
lemlib::ControllerSettings angularPID(7, // proportional gain (kP)
                                        0, // integral gain (kI)
                                        47, // derivative gain (kD)
                                        0, // anti windup
                                        0, // small error range, in inches
                                        0, // small error range timeout, in milliseconds
                                        0, // large error range, in inches
                                        0, // large error range timeout, in milliseconds
                                        0 // maximum acceleration (slew)
);




lemlib::ExpoDriveCurve throttleCurve(5, 12, 1.019); // deadband, minOutput, curve
lemlib::ExpoDriveCurve steerCurve(5, 8, 1.05); // deadband, minOutput, curve

lemlib::Chassis chassis(
    drivetrain,
    lateralPID,     // lateral PID settings
    angularPID,     // angular PID settings
    sensors,
    &throttleCurve,
    &steerCurve
);

// Note: do not run executable code at namespace scope. Set the chassis
// during runtime initialization (e.g., in initialize()).

pros::AIVision ai_vision(6);
