#include "main.h"
#include "crossBarrierDetection.h"
#include "globals.h" 
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/core/lv_obj_pos.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "auton/autonRoutines.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "robodash/views/image.hpp"
#include "robodash/views/selector.hpp"
#include "colorSort.h"
#include "warnings.h"
#include <string>
#include "MCL.h"
#include "auton/autonFunctions.h"
#include "distanceReset.h"
#include <atomic> 
#include <memory>
#include "motorDashboard.h"

LV_IMG_DECLARE(callogo);

static void update_alliance_btn(lv_obj_t* btn, Color newColor) {
    lv_obj_t* label = lv_obj_get_child(btn, 0);

    if (newColor == Color::RED) {
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);
        lv_label_set_text(label, "RED");
    } else {
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_label_set_text(label, "BLUE");
    }

    std::cout << "Alliance switched to: " 
              << (newColor == Color::RED ? "Red" : "Blue") 
              << std::endl;
}

static void alliance_btn_event_handler(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);

    if (allianceColor == RED) {
        allianceColor = Color::BLUE;
    } else {
        allianceColor = Color::RED;
    }

    lv_async_call([](void* user_data){
        lv_obj_t* btn = (lv_obj_t*)user_data;
        update_alliance_btn(btn, allianceColor);
    }, btn);
}

void create_alliance_selector() {
    lv_obj_t* btn = lv_btn_create(lv_layer_top());
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_size(btn, 80, 40);

    lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, "RED");
    lv_obj_center(label);

    lv_obj_add_event_cb(btn, alliance_btn_event_handler, LV_EVENT_CLICKED, nullptr);
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
rd::Image image = rd::Image(&callogo, "logo");

void initialize() {
    chassis.calibrate();
    //temp_warning();
    motor_disconnect_warning();
    distance_sensor_disconnect_warning();
    create_alliance_selector();
    init_motor_dashboard();

    color_sensor.set_integration_time(5);
    vertical_tracking_sensor.set_data_rate(5);
    horizontal_tracking_sensor.set_data_rate(5);


    double start_x = -51.25;
    double start_y = -18.5;
    double start_theta = 180.0;
    chassis.setPose(start_x, start_y, start_theta); 

    //MCL::StartMCL(start_x, start_y, start_theta);

    //pros::Task mcl_task(MCL::MonteCarlo);
    selector.focus();
    color_sensor.set_led_pwm(100);
    image.focus();
    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
            
            lemlib::Pose odomPose = chassis.getPose();
            //distancePose pose = distanceReset(false);
            console.printf("X: %.3f  Y: %.3f\n", odomPose.x, odomPose.y);
            console.printf("Theta: %.3f\n\n", odomPose.theta);
            //console.printf("Calculated Angle %.3f", calculateAngle(odomPose.theta));
            //console.printf("X_DSR: %.3f  Y_DSR: %.3f\n", pose.x, pose.y);
            
            
            //console.printf("X: %.2f  Y: %.2f\n", MCL::global_X, MCL::global_Y);
            //console.printf("Theta: %.2f\n", MCL::global_Theta);
            pros::delay(50);
        }
    });
}

void disabled() {
    image.focus();
    
}

void competition_initialize() {
    selector.focus();
}

float INCH_TO_METER = 0.0254f;
 float METER_TO_INCH = 39.3700787f;
float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
float gear_ratio = 4.0f / 3.0f;
 float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;

class Vector2 {
public:
    Vector2(float x, float y) : x(x), y(y) {}
    std::string latex() const {
        std::ostringstream oss;
        oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
        return oss.str();
    }

    float x;
    float y;
};

void collect_velocity_vs_voltage_data() {
    std::vector<float> inputs = {0.0f,  0.25f,  0.5f,  0.75f,  1.0f,  1.25f,  1.5f,  1.75f,  2.0f, 2.25f,
                                 2.5f,  2.75f,  3.0f,  3.25f,  3.5f,  3.75f,  4.0f,  4.25f,  4.5f, 4.75f,
                                 5.0f,  5.25f,  5.5f,  5.75f,  6.0f,  6.25f,  6.5f,  6.75f,  7.0f, 7.25f,
                                 7.5f,  7.75f,  8.0f,  8.25f,  8.5f,  8.75f,  9.0f,  9.25f,  9.5f, 9.75f,
                                 10.0f, 10.25f, 10.5f, 10.75f, 11.0f, 11.25f, 11.5f, 11.75f, 12.0f};
   

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
            v_sum += (std::fabs(leftMotors.get_actual_velocity()*rpm_to_mps_factor) + std::fabs(rightMotors.get_actual_velocity()*rpm_to_mps_factor)) / 2;
        }
        outputs.emplace_back( (v_sum / (float)n));
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
        auto speed = (std::fabs(leftMotors.get_actual_velocity()*rpm_to_mps_factor) + std::fabs(rightMotors.get_actual_velocity()*rpm_to_mps_factor)) / 2;
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
    std::cout << "\n" << std::endl;
    for (auto &s : logs2) std::cout << s, pros::delay(50);
}




void autonomous() {
    find_tracking_center(3.5, 5000);

}


void opcontrol() {
    midgoal_first = false;
    bool trapDoor_commanded = false;
    bool matchload_on = true;

    std::unique_ptr<pros::Task> longgoalTask = nullptr;
    int stall_timer = 0;
    int task_run_time = 0;

    static std::atomic<bool> macro_finished{false}; 
    static std::atomic<bool> macro_abort{false}; 

    enum class ScoringMode {
        NONE, INTAKE, OUTTAKE, MIDGOAL, LONGGOAL, MANUAL_UP, BRAKE
    };

    while (true) {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            if(matchloader.is_extended())
            {
                matchloader.retract();
            }
            else {
                matchloader.extend();
            }
        }
        ScoringMode scoringMode = ScoringMode::NONE;
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            intakeMotor.move_velocity(600);
            scoringMode = ScoringMode::BRAKE;

        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            outtakeMotor.move_velocity(600);
            scoringMode = ScoringMode::BRAKE;
        }


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            storageMotor.move_velocity(600);
            scoringMode = ScoringMode::BRAKE;
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            descore.retract();        
        } else {
            descore.extend();
        }

        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            scoringMode = ScoringMode::LONGGOAL;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            scoringMode = ScoringMode::MIDGOAL;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            scoringMode = ScoringMode::MANUAL_UP;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            scoringMode = ScoringMode::OUTTAKE;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            scoringMode = ScoringMode::INTAKE;
        }

        switch (scoringMode) {
            case ScoringMode::INTAKE:
                intake();
                break;
            case ScoringMode::OUTTAKE:
                outtake();
                break;
            case ScoringMode::MIDGOAL:
                score_midgoal();
                break;
            case ScoringMode::LONGGOAL:
                score_longgoal(600, allianceColor);
                break;
            case ScoringMode::MANUAL_UP:
                break;
            case ScoringMode::NONE:
                intake_stop();
                break;
            case ScoringMode::BRAKE:
                break;
        }

        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        
        if (longgoalTask == nullptr) {
            chassis.curvature(throttle, steer, false);
        }

        pros::delay(10);
    }
}