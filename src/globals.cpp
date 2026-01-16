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
                                              0.5, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              35 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0.0015, // integral gain (kI)
                                              24, // derivative gain (kD)
                                               3, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              75, // large error range timeout, in milliseconds
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
pros::adi::Pneumatics descore('D', false);

pros::Optical color_sensor(5);

pros::Vision vision_sensor(16);

Color allianceColor = Color::RED;
bool color_sort_enable = true;
/*
const float INCH_TO_METER = 0.0254f;
const float TRACK_WIDTH = 11.5f;

const float wheel_circumference = lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
const float gear_ratio = 4.0f / 3.0f;

const float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;


class Vector2 {
    public:
        Vector2(float x, float y) : x(x), y(y) {}
        std::string latex() const {
            std::ostringstream oss;
            oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
            return oss.str();
        }
        float x, y;
    };

void collect_velocity_vs_voltage_data() {
  std::vector<float> inputs = {
    0.0f,  0.5f,  1.0f,  1.5f,  2.0f,  2.5f,  3.0f,  3.5f,  4.0f,  4.5f,
    5.0f,  5.5f,  6.0f,  6.5f,  7.0f,  7.5f,  8.0f,  8.5f,  9.0f,  9.5f,
    10.0f, 10.5f, 11.0f, 11.5f, 12.0f
};

    std::vector<float> outputs = {0.f};
    outputs.reserve(inputs.size());

    float direction = 1;
    for (auto &input : inputs) {
        if (input == 0)
            continue;

        leftMotors.move_voltage(direction * input * 1000);
        rightMotors.move_voltage(-direction * input * 1000);

        pros::delay(1000);

        float v_sum = 0;
        int n;
        for (n = 0; n < 500; ++n) {
            v_sum += std::fabs(imu.get_gyro_rate().z * M_PI / 180);
        }

        outputs.emplace_back(v_sum / (float)n);

        auto v = input * direction * 1000;
        while (fabsf(v) > 0.5) {
            v *= 0.9;
            leftMotors.move_voltage(v);
            rightMotors.move_voltage(v);
            pros::delay(10);
        }
        if(input >= 6)
        {
            pros::delay(5000);
        }
        direction = -direction;
    }

    leftMotors.brake();
    rightMotors.brake();

    for (int i = 0; i < inputs.size(); ++i) {
        std::cout << Vector2(inputs[i], outputs[i]).latex() << ",";
    }
    std::cout << "\b" << std::endl;
}

void collect_voltage_step_data(float step_input, float duration) {
    std::vector<float> outputs = {};
    duration *= 100;
    outputs.reserve(duration);

    leftMotors.move_voltage(step_input * 1000);
    rightMotors.move_voltage(-step_input * 1000);

    for (int i = 0; i < duration; ++i) {
        auto speed = std::fabs(imu.get_gyro_rate().z * M_PI / 180);
        std::cout << Vector2((float)i / 100, speed).latex() << "," << std::flush;
        pros::delay(10);
    }

    leftMotors.brake();
    rightMotors.brake();
    std::cout << "\b" << std::endl;
}

void find_tracking_center(float turnVoltage, uint32_t time_ms) {
    chassis.setPose(0, 0, 180);

    std::vector<std::string> logs;
    std::vector<std::string> logs2;

    uint32_t start = pros::millis();
    while (pros::millis() - start < time_ms)
    {
        leftMotors.move_voltage(turnVoltage * 1000);
        rightMotors.move_voltage(-turnVoltage * 1000);

        auto pose = chassis.getPose(false); 
        logs.push_back("(" + std::to_string(pose.x) + "," + std::to_string(pose.y) + "),");
        logs2.push_back(std::to_string(pose.theta) + ",");

        pros::delay(20);
    }
    leftMotors.brake();
    rightMotors.brake();

    for (auto &s : logs) std::cout << s, pros::delay(50);
    std::cout << "/n";
    for (auto &s : logs2) std::cout << s, pros::delay(50);
}
const VelocityControllerConfig config{
5.8432642308,
0.213526937516,
1.14429410811,
0.906356177095,
0.347072436421,
11.4953431776,
54.5797495382,
};



void velocity_test(const VelocityControllerConfig &config, float max_velocity, int duration, int acceleration_time) {
    duration /= 10;
    acceleration_time /= 10;

    VoltageController controller(config.kV, config.KA_straight, config.KA_turn, config.KS_straight, config.KS_turn, config.KP_straight, config.KI_straight, 99999, 12.8 * INCH_TO_METER);

    //std::cout << "\\left[";

    int i;
    for (i = 0; i < duration; ++i) {
        auto v_d = max_velocity * fminf(fminf(1, (float)i / (float)acceleration_time),
                                        (float)(duration - i) / (float)acceleration_time);
        //auto speed = (leftMotors.get_actual_velocity() + rightMotors.get_actual_velocity()) / 2;
        //std::cout << Vector2(i * 0.01f, v_d).latex() << ",";
        float velocity = std::fabs(imu.get_gyro_rate().z * M_PI/180.0f);
        std::cout << Vector2(i * 0.01f, velocity).latex() << ",";
        std::cout.flush();
        auto voltage = controller.update(0, v_d, leftMotors.get_actual_velocity() * rpm_to_mps_factor, rightMotors.get_actual_velocity() * rpm_to_mps_factor);
        leftMotors.move_voltage(voltage.leftVoltage * 1000);
        rightMotors.move_voltage(voltage.rightVoltage * 1000);

        pros::delay(10);
    }

    leftMotors.brake();
    rightMotors.brake();

    
    std::cout << "\b" << std::endl;
}*/






