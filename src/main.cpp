#include "main.h"
#include "globals.h" 
#include "liblvgl/core/lv_obj_pos.h"
#include "pros/misc.h"
#include "auton/autonRoutines.h"
#include "auton/autonFunctions.h"
#include "robodash/views/selector.hpp"
#include "distanceReset.h"
#include "colorSort.h"
#include "warnings.h"
#include <string>
#include "MCL.h"

Alliance currentAlliance = RED_ALLIANCE;

static void alliance_btn_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    lv_obj_t * label = lv_obj_get_child(btn, 0);

    if(code == LV_EVENT_CLICKED) {
        if(currentAlliance == RED_ALLIANCE) {
            currentAlliance = BLUE_ALLIANCE;
            lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), 0);
            lv_label_set_text(label, "BLUE");
        } else {
            currentAlliance = RED_ALLIANCE;
            lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);
            lv_label_set_text(label, "RED");
        }
        
        std::cout << "Alliance switched to: " << (currentAlliance == RED_ALLIANCE ? "Red" : "Blue") << std::endl;
    }
}

void create_alliance_selector() {
    lv_obj_t * btn = lv_btn_create(lv_layer_top());
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_size(btn, 80, 40);
    lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "RED");
    lv_obj_center(label);
    lv_obj_add_event_cb(btn, alliance_btn_event_handler, LV_EVENT_ALL, NULL);
}


rd::Selector selector({
    {"Right", right_auton},
    {"Left", left_auton},
    {"Carry", carry_auton},
    {"Elim", elim_auton},
    {"AWP", awp_auton},
    {"Skills", skills_auton}
});

rd::Console console;

void initialize() {
    chassis.calibrate();
    //temp_warning();
    //motor_disconnect_warning();
    //distance_sensor_disconnect_warning();

    //precompute_auton_paths();
    /*
   chassis.setPose(-63.5, -18.5, 180);
   MCL::StartMCL(-63.5, -18.5, 180);
   pros::Task mclTask(MCL::MonteCarlo);
    */
    console.focus();
    create_alliance_selector();
    pros::Task screen_task([&]() {
        lemlib::Pose pose{0,0,0};
        while (true) {
            console.clear();
            pose = chassis.getPose();
            //distancePose dpose = distanceReset();
            console.printf("X: %f\n", pose.x);
            console.printf("Y: %f\n", pose.y);
            console.printf("Theta: %f\n", pose.theta);

            /*
            DISTANCE SENSOR TESTING
            console.printf("D X: %f\n", dpose.x);
            console.printf("D Y: %f\n", dpose.y);
            console.printf("Using X: %d\n", dpose.using_odom_x);
            console.printf("Using Y: %d\n", dpose.using_odom_y);
            */
            console.printf("X MCL: %f\n", MCL::X);
            console.printf("Y MCL: %f\n", MCL::Y);

            pros::delay(100);
        }
    });
}

void disabled() {
    console.focus();
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
        5.0f,  5.25f, 5.5f,  5.75f, 6.0f,  6.25f, 6.5f,  6.75f, 7.0f,
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
    rightMotors.move_voltage(-step_input * 1000);

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
    chassis.setPose(0, 0, 180);

    std::vector<std::string> logs;
    std::vector<std::string> logs2;

    uint32_t start = pros::millis();
    while (pros::millis() - start < time_ms)
    {
        leftMotors.move_voltage(turnVoltage * 1000);
        rightMotors.move_voltage(-turnVoltage * 1000);

        auto pose = chassis.getPose(false);  // don't estimate
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


void autonomous() {
   chassis.setPose(0,0,0);
   //collect_velocity_vs_voltage_data();
   collect_voltage_step_data(5.82062493625, 2);
   
}

/*

Straight:
KV = 5.19338427813
1.26552223944
0.676257433253
11.6978629947
46.9504105993

Turn:
7.22576300573
1.36207946996
1.54163120695
19.8980417469
106.56124453

*/

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
            intake_to_basket();
        }
        else
        {
            intake_stop();
            //resting_state();
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            throttle = 0;
            steer = 0;
            leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            leftMotors.brake();
            rightMotors.brake();
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            matchload.toggle();
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            trapDoor.toggle();
        }
        else {
            leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }



        chassis.curvature(throttle, steer, false);
        pros::delay(20);
    }
}



