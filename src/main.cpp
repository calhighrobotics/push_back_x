#include <vector>
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

const std::vector<float> path_0_linear = {
    0.000000f, 0.075000f, 0.077386f, 0.079774f, 0.152726f, 0.228122f, 0.230530f, 0.304995f, 
    0.380671f, 0.383180f, 0.458002f, 0.533756f, 0.536368f, 0.611864f, 0.614470f, 0.689245f, 
    0.764631f, 0.767254f, 0.842177f, 0.844848f, 0.920091f, 0.922743f, 0.997553f, 1.000244f, 
    1.021637f, 1.019776f, 0.964552f, 0.962899f, 0.907934f, 0.906467f, 0.851642f, 0.850340f, 
    0.795709f, 0.794557f, 0.739944f, 0.738923f, 0.684467f, 0.683568f, 0.629179f, 0.574811f, 
    0.574122f, 0.520326f, 0.519739f, 0.465947f, 0.465432f, 0.411219f, 0.357043f, 0.356679f, 
    0.302571f, 0.302274f, 0.248263f, 0.194406f, 0.194230f, 0.140708f, 0.140586f, 0.087998f, 
    0.040160f, 0.040137f, 0.071444f, 0.071527f, 0.149307f, 0.227087f, 0.263537f, 0.341317f, 
    0.367624f, 0.445404f, 0.472697f, 0.550478f, 0.628258f, 0.683713f, 0.761493f, 0.790224f, 
    0.868004f, 0.897011f, 0.974791f, 1.004030f, 1.081810f, 1.111302f, 1.189082f, 1.218792f, 
    1.296572f, 1.326467f, 1.404247f, 1.434374f, 1.512154f, 1.542409f, 1.620189f, 1.650530f, 
    1.728310f, 1.758819f, 1.836599f, 1.914379f, 1.974102f, 1.940699f, 1.894286f, 1.847035f, 
    1.798581f, 1.748732f, 1.697485f, 1.645043f, 1.591827f, 1.538476f, 1.485831f, 1.434890f, 
    1.386747f, 1.342510f, 1.303224f, 1.269806f, 1.242986f, 1.223292f, 1.211035f, 1.206324f, 
    1.209080f, 1.219045f, 1.235801f, 1.258781f, 1.287283f, 1.320488f, 1.357484f, 1.397303f, 
    1.438962f, 1.481509f, 1.524065f, 1.565863f, 1.606272f, 1.644804f, 1.681121f, 1.715016f, 
    0.000000f
};

const std::vector<float> path_1_linear = {
    1.831190f, 1.784325f, 1.726743f, 1.655960f, 1.569429f, 1.465176f, 1.342950f, 1.205734f, 
    1.060751f, 0.918630f, 0.790250f, 0.683146f, 0.600063f, 0.540066f, 0.500564f, 0.478888f, 
    0.473049f, 0.481999f, 0.505586f, 0.544402f, 0.599508f, 0.671983f, 0.749763f, 0.827543f, 
    0.905323f, 0.983103f, 1.060883f, 1.138663f, 1.216443f, 1.294008f, 1.270340f, 1.247207f, 
    1.223645f, 1.200012f, 1.176947f, 1.153869f, 1.130342f, 1.106767f, 1.083742f, 1.060213f, 
    1.036670f, 1.013667f, 0.990159f, 0.966658f, 0.943653f, 0.920672f, 0.897189f, 0.873707f, 
    0.850722f, 0.827273f, 0.803782f, 0.780826f, 0.757377f, 0.733908f, 0.710962f, 0.688003f, 
    0.664591f, 0.641124f, 0.618188f, 0.594792f, 0.571323f, 0.548393f, 0.525466f, 0.502092f, 
    0.478618f, 0.455697f, 0.432340f, 0.408872f, 0.385957f, 0.362631f, 0.339145f, 0.316248f, 
    0.293343f, 0.270047f, 0.246564f, 0.223681f, 0.200420f, 0.176939f, 0.154091f, 0.130898f, 
    0.107453f, 0.084711f, 0.062083f, 0.039541f, 0.018155f, 0.000000f, 0.077780f, 0.155560f, 
    0.233340f, 0.311120f, 0.388900f, 0.466680f, 0.544460f, 0.622240f, 0.700020f, 0.777800f, 
    0.855580f, 0.933360f, 1.011140f, 1.088920f, 1.166700f, 1.244480f, 1.322260f, 1.400040f, 
    1.477820f, 1.555600f, 1.633380f, 1.711160f, 1.788940f, 1.866720f, 1.944500f, 2.022280f, 
    2.100060f, 2.104296f, 2.101742f, 2.099241f, 2.096781f, 2.094351f, 2.091940f, 2.089539f, 
    2.087138f, 2.084730f, 2.082305f, 2.079855f, 2.077373f, 2.074849f, 2.072277f, 2.069649f, 
    2.066955f, 2.064188f, 2.061340f, 2.058400f, 2.055361f, 2.052211f, 2.048941f, 2.045541f, 
    2.041998f, 2.038299f, 2.011858f, 1.932514f, 1.853103f, 1.773620f, 1.694057f, 1.614408f, 
    1.534661f, 1.454807f, 1.374833f, 1.294723f, 1.214459f, 1.134020f, 1.053376f, 0.972494f, 
    0.891330f, 0.809828f, 0.727907f, 0.645459f, 0.562327f, 0.478267f, 0.392861f, 0.305328f, 
    0.213846f, 0.111624f
};

// Path 0: Angular Velocities (rad/s)
const std::vector<float> path_0_angular = {
    0.000000f, 0.120614f, 0.125006f, 0.129460f, 0.250047f, 0.378467f, 0.387643f, 0.522117f, 
    0.666488f, 0.686360f, 0.843246f, 1.015020f, 1.054014f, 1.248709f, 1.303069f, 1.526564f, 
    1.778046f, 1.874330f, 2.172681f, 2.302822f, 2.663104f, 2.836187f, 3.270214f, 3.494279f, 
    3.802005f, 4.032434f, 4.027086f, 4.228385f, 4.163826f, 4.320137f, 4.186860f, 4.288377f, 
    4.087307f, 4.133543f, 3.874496f, 3.873765f, 3.575124f, 3.541096f, 3.221827f, 2.902995f, 
    2.851162f, 2.538600f, 2.485614f, 2.184842f, 2.136372f, 1.849910f, 1.576901f, 1.545328f, 
    1.288935f, 1.265436f, 1.024174f, 0.792642f, 0.782542f, 0.561966f, 0.556538f, 0.346418f, 
    0.157693f, 0.157199f, 0.278535f, 0.277573f, 0.573817f, 0.859792f, 0.980369f, 1.240550f, 
    1.302493f, 1.529037f, 1.568163f, 1.753402f, 1.908455f, 1.970284f, 2.066849f, 2.013164f, 
    2.060018f, 1.975891f, 1.977121f, 1.867327f, 1.828769f, 1.698746f, 1.626441f, 1.480760f, 
    1.379598f, 1.221071f, 1.093758f, 0.922754f, 0.768917f, 0.582528f, 0.397910f, 0.189272f, 
    -0.035639f, -0.279062f, -0.561495f, -0.890432f, -1.262524f, -1.600722f, -1.937860f, -2.284873f, 
    -2.643257f, -3.013034f, -3.392563f, -3.778355f, -4.165026f, -4.545393f, -4.910840f, -5.251948f, 
    -5.559294f, -5.824327f, -6.040107f, -6.201863f, -6.307141f, -6.355733f, -6.349352f, -6.291206f, 
    -6.185621f, -6.037622f, -5.852704f, -5.636613f, -5.395225f, -5.134447f, -4.860103f, -4.577806f, 
    -4.292840f, -4.009996f, -3.733458f, -3.466675f, -3.212348f, -2.972384f, -2.747978f, -2.746201f, 
    0.000000f
};

const std::vector<float> path_1_angular = {
    -2.384643f, -2.812989f, -3.349483f, -4.021296f, -4.855364f, -5.868451f, -7.048977f, -8.334518f, 
    -9.602725f, -10.700188f, -11.505576f, -11.978070f, -12.151621f, -12.094702f, -11.874341f, -11.541250f, 
    -11.128050f, -10.653962f, -10.128982f, -9.557714f, -8.942410f, -8.285260f, -7.492133f, -6.629900f, 
    -5.793393f, -5.031149f, -4.360722f, -3.782515f, -3.288893f, -2.868907f, -2.358395f, -1.972654f, 
    -1.672933f, -1.435615f, -1.244932f, -1.088825f, -0.958941f, -0.849897f, -0.757755f, -0.678540f, 
    -0.610152f, -0.550953f, -0.498873f, -0.453015f, -0.412619f, -0.376676f, -0.344364f, -0.315379f, 
    -0.289440f, -0.265857f, -0.244474f, -0.225191f, -0.207485f, -0.191311f, -0.176631f, -0.163157f, 
    -0.150671f, -0.139165f, -0.128658f, -0.118861f, -0.109781f, -0.101464f, -0.093743f, -0.086488f, 
    -0.079718f, -0.073492f, -0.067610f, -0.062090f, -0.056997f, -0.052155f, -0.047578f, -0.043340f, 
    -0.039333f, -0.035483f, -0.031800f, -0.028363f, -0.025027f, -0.021797f, -0.018758f, -0.015774f, 
    -0.012841f, -0.010057f, -0.007335f, -0.004657f, -0.002135f, -0.000000f, -0.009092f, -0.017963f, 
    -0.026451f, -0.034402f, -0.041671f, -0.048126f, -0.053645f, -0.058125f, -0.061474f, -0.063619f, 
    -0.064498f, -0.064064f, -0.062285f, -0.059137f, -0.054607f, -0.048689f, -0.041381f, -0.032686f, 
    -0.022609f, -0.011152f, 0.001685f, 0.015907f, 0.031522f, 0.048553f, 0.067026f, 0.086985f, 
    0.108485f, 0.126537f, 0.143867f, 0.160913f, 0.177753f, 0.194460f, 0.211099f, 0.227735f, 
    0.244428f, 0.261237f, 0.278218f, 0.295428f, 0.312922f, 0.330755f, 0.348984f, 0.367665f, 
    0.386857f, 0.406619f, 0.427014f, 0.448108f, 0.469968f, 0.492666f, 0.516279f, 0.540886f, 
    0.566574f, 0.593433f, 0.614336f, 0.617903f, 0.619419f, 0.618783f, 0.615895f, 0.610656f, 
    0.602972f, 0.592756f, 0.579932f, 0.564436f, 0.546220f, 0.525252f, 0.501520f, 0.475031f, 
    0.445815f, 0.413922f, 0.379416f, 0.342378f, 0.302887f, 0.261004f, 0.216724f, 0.169860f, 
    0.119672f, 0.062605f
};

pros::Controller controller(CONTROLLER_MASTER);

pros::MotorGroup rightMotors({11,12,13});
pros::MotorGroup leftMotors({-18,-19,-20});

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10, 3.25, 120, 2);

pros::IMU imu(15);

pros::Rotation horizontal_tracking_sensor(-6);
pros::Rotation vertical_tracking_sensor(-16);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, 3.75, 7.1*2.54/100, 1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, 3.75, 0.1*2.54/100,1);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);


std::vector<double> calculate_velocities(double linear, double angular)
{   
    double right_vel = linear + 0.5*angular*0.254;
    double left_vel = linear - 0.5*angular*0.254;
    return std::vector<double>{right_vel, left_vel};
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.

 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    double target_linear_vel;
    double target_angular_vel;
    std::vector<double> velocities;
    const int STEP_INTERVAL = 10;

    const double MAX_PATH_VELOCITY = 2.12;
    
    for(int i = 0; i < path_0_linear.size(); i++)
    {
        target_linear_vel = path_0_linear.at(i);
        target_angular_vel = path_0_angular.at(i);
        velocities = calculate_velocities(target_linear_vel, target_angular_vel);
        double right_voltage = (velocities.at(0) / MAX_PATH_VELOCITY) * 12000.0;
        double left_voltage = (velocities.at(1) / MAX_PATH_VELOCITY) * 12000.0;

        rightMotors.move_voltage(right_voltage);
        leftMotors.move_voltage(left_voltage);

       pros::delay(10);
    }

    rightMotors.move_voltage(0);
    leftMotors.move_voltage(0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
    while (true)
    {
        chassis.turnToHeading(90, 3000);
    }
}