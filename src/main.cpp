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
#include "robodash/views/selector.hpp"
#include "colorSort.h"
#include "warnings.h"
#include <string>
#include "MCL.h"
#include "auton/autonFunctions.h"
#include "distanceReset.h"
#include <atomic> // For std::atomic
#include <memory> // For std::unique_ptr and std::make_unique


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

void initialize() {
    chassis.calibrate();
    //temp_warning();
    motor_disconnect_warning();
    distance_sensor_disconnect_warning();
    create_alliance_selector();
    color_sensor.set_integration_time(5);
    vertical_tracking_sensor.set_data_rate(5);
    horizontal_tracking_sensor.set_data_rate(5);

    double start_x = -51.25;
    double start_y = -18.5;
    double start_theta = 180.0;
    chassis.setPose(start_x, start_y, start_theta); 

    //MCL::StartMCL(start_x, start_y, start_theta);

    //pros::Task mcl_task(MCL::MonteCarlo);
    chassis.setPose(0,0,0);
    selector.focus();
    color_sensor.set_led_pwm(100);
    chassis.setPose(-50, 0, 270);
    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
            
            lemlib::Pose odomPose = chassis.getPose();
            //distancePose pose = distanceReset(false);
            console.printf("X: %.3f  Y: %.3f\n", odomPose.x, odomPose.y);
            console.printf("Theta: %.3f\n\n", odomPose.theta);
            //console.printf("Calculated Angle %.3f", calculateAngle(odomPose.theta));
            //console.printf("X_DSR: %.3f  Y_DSR: %.3f\n", pose.x, pose.y);
            
            
            console.printf("X: %.2f  Y: %.2f\n", MCL::global_X, MCL::global_Y);
            console.printf("Theta: %.2f\n", MCL::global_Theta);
            /*
            console.printf("Conf: %.1f%%\n", MCL::global_Confidence * 100.0);

            double error_dist = std::hypot(odomPose.x - MCL::global_X, odomPose.y - MCL::global_Y);
            console.printf("Delta Dist: %.2f in\n", error_dist);
            */
            pros::delay(50);
        }
    });
}

void disabled() {
    selector.focus();
    
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
    bool matchload_on = false;

    std::unique_ptr<pros::Task> longgoalTask = nullptr;
    int stall_timer = 0;
    int task_run_time = 0;

    static std::atomic<bool> macro_finished{false}; 
    static std::atomic<bool> macro_abort{false}; 

    enum class ScoringMode {
        NONE, INTAKE, OUTTAKE, MIDGOAL, LONGGOAL, MANUAL_UP
    };

    while (true) {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            matchload_state(!matchloader.is_extended());
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            color_sort_enable = !color_sort_enable;
            controller.rumble(".");
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            
            if (longgoalTask != nullptr) {
                macro_abort = true;
                chassis.cancelMotion(); 
            }
    
            else {
                stall_timer = 0;
                task_run_time = 0;
                macro_finished = false;
                macro_abort = false; 

                longgoalTask = std::make_unique<pros::Task>([] {
                    distancePose pose = distanceReset(true);

                    if (!pose.using_odom_x && !pose.using_odom_y &&
                        std::abs(std::abs(pose.x) - (22 + longgoal_offset)) < 4 &&
                        std::abs(std::abs(pose.y) - 47.5) < 4) {
                        
                        int x_mult = pose.x < 0 ? -1 : 1;
                        int y_mult = pose.y < 0 ? -1 : 1;

                        chassis.moveToPoint(49 * x_mult, 47.5 * y_mult, 3000, {.earlyExitRange = 2});
                        if (macro_abort) { macro_finished = true; return; } 

                        descore.retract();
                        chassis.waitUntilDone();
                        if (macro_abort) { macro_finished = true; return; }

                        chassis.moveToPose((22 + longgoal_offset) * x_mult,
                                        (47.5 * y_mult) + 13.5, x_mult > 0 ? 90 : 270, 1500,
                                        {.forwards = false, .lead = 0.2, .minSpeed = 20, .earlyExitRange = 8});
                        if (macro_abort) { macro_finished = true; return; }

                        chassis.moveToPose(-16 * x_mult,
                                        (47.5 * y_mult) + 12.5, x_mult > 0 ? 90 : 270, 1500,
                                        {.forwards = false, .lead = 0.3});
                    }
                    
                    macro_finished = true; 
                });
            }
        }
        if (longgoalTask != nullptr) {
            task_run_time += 10;

    
            if (macro_finished) {
                longgoalTask = nullptr; 
            }

            else if (std::abs(throttle) > 15 || std::abs(steer) > 15) {
                macro_abort = true;
                chassis.cancelMotion();
            }
        
            else if (task_run_time > 3500) {
                macro_abort = true;
                chassis.cancelMotion();
                controller.rumble("-"); 
            }
    
            else if (task_run_time > 500) {
                double left_vel = std::abs(leftMotors.get_actual_velocity());
                double right_vel = std::abs(rightMotors.get_actual_velocity());

                if (left_vel < 10.0 && right_vel < 10.0) {
                    stall_timer += 10;
                } else {
                    stall_timer = 0; 
                }

                if (stall_timer > 500) {
                    macro_abort = true;
                    chassis.cancelMotion();
                    controller.rumble("- -"); 
                }
            }
        } else {
            stall_timer = 0;
            task_run_time = 0;
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            descore.retract();        
        } else {
            descore.extend();
        }

        ScoringMode scoringMode = ScoringMode::NONE;
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
        }

        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        
        if (longgoalTask == nullptr) {
            chassis.curvature(throttle, steer, false);
        }

        pros::delay(10);
    }
}