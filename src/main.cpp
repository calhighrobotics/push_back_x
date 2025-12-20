#include "main.h"
#include "globals.h" 
#include "liblvgl/core/lv_obj_pos.h"
#include "pros/misc.h"
#include "auton/autonRoutines.h"
#include "auton/autonFunctions.h"
#include "velocityController.h"
#include "pathFollowing/ramsete.cpp"
#include "pathFollowing/paths.cpp"
#include "robodash/views/selector.hpp"
#include "distanceReset.h"
#include "colorSort.h"
#include "warnings.h"
#include <string>
#include "MCL.h"

const VelocityControllerConfig config{
    12.4370890785,
    0.803031225567,
    0.664537661342,
    0.472796490892,
    0.236548087393,
    25.2621164319,
    524.703492373,
};

rd::Selector selector({
    {"Right", right_auton},
    {"Left", left_auton},
    {"Carry", carry_auton},
    {"Elim", elim_auton},
    {"AWP", awp_auton},
    {"Skills", skills_auton}
});

rd::Console console;


RamsetePathFollower ramsete(config, 2, 0.7);
void initialize() {

    chassis.calibrate();
    //temp_warning();
    //motor_disconnect_warning();
    //distance_sensor_disconnect_warning();
    //MCL::StartMCL(0, 0, 0);
    //pros::Task mclTask(MCL::MonteCarlo);
    std::vector<std::string> paths = {test_path, right_1};
    ramsete.precompute_paths(paths);

    pros::Task screen_task([&]() {
        chassis.calibrate();
        lemlib::Pose pose{0,0,0};
        while (true) {
            console.clear();
            pose = chassis.getPose();
            console.printf("X: %f", pose.x);
            console.printf("Y: %f", pose.y);
            console.printf("Theta: %f", pose.theta);
            pros::delay(20);
        }
    });
}

void disabled() {
    //selector.focus();
}

void competition_initialize() {
}

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
        0.0f,  0.25f, 0.5f,  0.75f, 1.0f,  1.25f, 1.5f,  1.75f, 2.0f,  2.25f,
        2.5f,  2.75f, 3.0f,  3.25f, 3.5f,  3.75f, 4.0f,  4.25f, 4.5f,  4.75f,
        5.0f,  5.25f, 5.5f,  5.75f, 6.0f,  6.25f, 6.5f,  6.75f, 7.0f,  7.25f,
        7.5f,  7.75f, 8.0f,  8.25f, 8.5f,  8.75f, 9.0f,  9.25f, 9.5f,  9.75f,
        10.0f, 10.25f, 10.5f, 10.75f, 11.0f, 11.25f, 11.5f, 11.75f, 12.0f
    };

    std::vector<float> outputs = {0.f};
    outputs.reserve(inputs.size());

    float direction = 1;
    for (auto &input : inputs) {
        if (input == 0)
            continue;

        leftMotors.move_voltage(direction * input * 1000);
        rightMotors.move_voltage(direction * input * 1000);

        pros::delay(1000);

        float v_sum = 0;
        int n;
        for (n = 0; n < 500; ++n) {
            v_sum += (std::fabs(leftMotors.get_actual_velocity() * rpm_to_mps_factor) +
                      std::fabs(rightMotors.get_actual_velocity() * rpm_to_mps_factor)) / 2;
        }

        outputs.emplace_back(v_sum / (float)n);

        auto v = input * direction * 1000;
        while (fabsf(v) > 0.5) {
            v *= 0.9;
            leftMotors.move_voltage(v);
            rightMotors.move_voltage(v);
            pros::delay(10);
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

void collect_voltage_step_data(float step_input, unsigned int duration) {
    std::vector<float> outputs = {};
    duration *= 100;
    outputs.reserve(duration);

    leftMotors.move_voltage(step_input * 1000);
    rightMotors.move_voltage(step_input * 1000);

    for (int i = 0; i < duration; ++i) {
        auto speed = (std::fabs(leftMotors.get_actual_velocity() * rpm_to_mps_factor) +
                      std::fabs(rightMotors.get_actual_velocity() * rpm_to_mps_factor)) / 2;
        std::cout << Vector2((float)i / 100, speed).latex() << "," << std::flush;
        pros::delay(10);
    }

    leftMotors.brake();
    rightMotors.brake();
    std::cout << "\b" << std::endl;
}

void find_tracking_center(float turnVoltage, uint32_t time_ms) {
    chassis.setPose(0, 0, 0);

    std::vector<std::string> logs;
    std::vector<std::string> logs2;

    uint32_t start = pros::millis();
    while (pros::millis() - start < time_ms)
    {
        leftMotors.move_voltage(turnVoltage * 1000);
        rightMotors.move_voltage(-turnVoltage * 1000);

        auto pose = chassis.getPose(false);  // don't estimate
        logs.push_back(std::to_string(pose.x) + "," + std::to_string(pose.y) + ",");
        logs2.push_back(std::to_string(pose.theta) + ",");

        pros::delay(20);
    }
    leftMotors.brake();
    rightMotors.brake();

    for (auto &s : logs) std::cout << s, pros::delay(50);
    for (auto &s : logs2) std::cout << s, pros::delay(50);
}


void autonomous() {
   chassis.setPose(0,0,0);
   find_tracking_center(6, 4000);
   //collect_velocity_vs_voltage_data();
}

void opcontrol() {
    while(true)
    {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            outtake();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            score_longgoal();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            score_midgoal();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
        {
            intake();
            topMotor.move_voltage(-6000);
        }
        else
        {
            intake_stop();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            A.toggle();
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            B.toggle();
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            C.toggle();
        }



        chassis.curvature(throttle, steer, false);
        pros::delay(20);
    }
}



