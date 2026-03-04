#include "main.h"
#include "crossBarrierDetection.h"
#include "globals.h" 
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
#include "visionAlignment.h"
#include "auton/autonFunctions.h"
#include "distanceReset.h"
#include "ltv.h"
#include "paths.h"
#include "ramsete.h"


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
    temp_warning();
    motor_disconnect_warning();
    distance_sensor_disconnect_warning();
    create_alliance_selector();
    double start_x = -51.25;
    double start_y = -18.5;
    double start_theta = 180.0;
    chassis.setPose(start_x, start_y, start_theta); 

    if(pros::battery::get_capacity() <= 20)
    {   
        controller.rumble("..");
        controller.print(0, 0, "Battery Low%i", (int)pros::battery::get_capacity());
        pros::delay(1000);
    }

    //MCL::StartMCL(start_x, start_y, start_theta);

    //pros::Task mcl_task(MCL::MonteCarlo);

    console.focus();
    color_sensor.set_led_pwm(100);
    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
            
            lemlib::Pose odomPose = chassis.getPose();
            distancePose pose = distanceReset(false);
            console.printf("X: %.3f  Y: %.3f\n", odomPose.x, odomPose.y);
            console.printf("Theta: %.3f\n\n", odomPose.theta);
            console.printf("X_DSR: %.3f  Y_DSR: %.3f\n", pose.x, pose.y);
            
            /*
            console.printf("X: %.2f  Y: %.2f\n", MCL::global_X, MCL::global_Y);
            console.printf("Theta: %.2f\n", MCL::global_Theta);
        
            console.printf("Conf: %.1f%%\n", MCL::global_Confidence * 100.0);

            double error_dist = std::hypot(odomPose.x - MCL::global_X, odomPose.y - MCL::global_Y);
            console.printf("Delta Dist: %.2f in\n", error_dist);
            */
            pros::delay(50);
        }
    });
}

void disabled() {
    console.focus();
}

void competition_initialize() {
    console.focus();
}

void autonomous() {
    skills_auton();
}

void opcontrol()
{
    midgoal_first = false;
    std::cout << allianceColor << std::endl;
    bool trapDoor_commanded = false;
    bool matchload_on = false;

    enum class ScoringMode {
        NONE,
        INTAKE,
        OUTTAKE,
        MIDGOAL,
        LONGGOAL,
        MANUAL_UP
    };

    while (true) {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            trapDoor_commanded = !trapDoor_commanded;
            if (trapDoor_commanded)
                trapDoor.extend();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            if (matchload.is_extended()) {
                matchload.retract();
                matchload_on = false;
            } else {
                matchload.extend();
                matchload_on = true;
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            color_sort_enable = !color_sort_enable;
            controller.rumble(".");
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            basket.toggle();
            if (!scoringBand.is_extended())
                scoringBand.extend();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            scoringBand.toggle();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            midgoal_first = true;
            if(!trapDoor.is_extended())
            {
                trapDoor.extend();
            }
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            descore.retract();        
        }
        else {
            descore.extend();
        }

        ScoringMode scoringMode = ScoringMode::NONE;

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            scoringMode = ScoringMode::LONGGOAL;
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            scoringMode = ScoringMode::MIDGOAL;
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            scoringMode = ScoringMode::MANUAL_UP;
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            scoringMode = ScoringMode::OUTTAKE;
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            scoringMode = ScoringMode::INTAKE;
        }

        switch (scoringMode) {
            case ScoringMode::INTAKE:
                intake();
                break;

            case ScoringMode::OUTTAKE:
                outtake(8500);
                if (intakeFunnel.is_extended())
                    intakeFunnel.retract();
                low_ramp_down_time += 10;
                break;

            case ScoringMode::MIDGOAL:
                score_midgoal();
                ramp_up_time += 10;
                break;

            case ScoringMode::LONGGOAL:
                score_longgoal(12000, allianceColor);
                break;

            case ScoringMode::MANUAL_UP:
                topMotor.move(12000);
                intakeMotor.move(-12000);
                break;

            case ScoringMode::NONE:
                ramp_up_time = 0;
                resting_state(trapDoor_commanded);
                midgoal_first = false;
                intakeFunnel.extend();
                low_ramp_down_time = 0;
                if(!color_sort_enable && scoringBand.is_extended())
                {
                    scoringBand.retract();
                }
                break;
        }

        if (color_sort_enable)
            blockBlocker.extend();
        else
            blockBlocker.retract();

        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        chassis.curvature(throttle, steer, false);

        pros::delay(10);
    }
}