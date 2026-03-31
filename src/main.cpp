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






void autonomous() {
    selector.run_auton();
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