#include "main.h"
#include "crossBarrierDetection.h"
#include "globals.h" 
#include "liblvgl/core/lv_obj_pos.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "auton/autonRoutines.h"
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
    console.focus();
    //calibrate_vision();
    create_alliance_selector();
    /*
    chassis.setPose(-51.25, -18.5, 180);
    MCL::StartMCL(-51.25, -18.5, 180);
    pros::Task mcl_task(MCL::MonteCarlo);
    */
    chassis.setPose(-22 - longgoal_offset, 47.5, 180);
    console.focus();
    pros::Task screen_task([&]() {
        lemlib::Pose pose{0,0,0};
        while (true) {
            console.clear();
            pose = chassis.getPose();

            
            //distancePose dpose = distanceReset(false, 1);
            console.printf("X: %f\n", pose.x);
            console.printf("Y: %f\n", pose.y);
            console.printf("Theta: %f\n", pose.theta);
            
            /*
            console.printf("D X: %f\n", dpose.x);
            console.printf("D Y: %f\n", dpose.y);
            console.printf("Using X: %d\n", dpose.using_odom_x);
            console.printf("Using Y: %d\n", dpose.using_odom_y);
            */
            /*
            console.printf("MCL X: %f\n", MCL::global_X);
            console.printf("MCL Y: %f\n", MCL::global_Y);
            console.printf("MCL THETA %f\n", MCL::global_Theta);
            */
        
            
            
            pros::delay(20);
        }
    });
}

void disabled() {
}

void competition_initialize() {
}


void distanceCalibration() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -4, 2000, {.forwards = false});
    chassis.waitUntilDone();

    std::vector<pros::Distance*> sensors = {
        &frontDistance, &leftDistance, &backDistance, &rightDistance
    };
    std::vector<std::string> names = {
        "Front", "Left", "Back", "Right"
    };

    std::vector<int> sweepAngles = {0, 30, 45};

    for (int s = 0; s < sensors.size(); s++) {
        int baseHeading = s * 90;

        chassis.turnToHeading(baseHeading, 1000);
        chassis.waitUntilDone();
        pros::delay(300);

        for (int repeat = 0; repeat < 5; repeat++) {
            for (int angle : sweepAngles) {
                int target = baseHeading + angle;

                chassis.turnToHeading(target, 700);
                chassis.waitUntilDone();
                pros::delay(200);

                int dist = sensors[s]->get();
                std::cout << names[s]
                          << " @ +" << angle
                          << " deg: "
                          << dist << " mm"
                          << std::endl;
            }

            chassis.turnToHeading(baseHeading, 700);
            chassis.waitUntilDone();
            pros::delay(400);
        }
    }
}

void autonomous() {
    skills_auton();
    //elim_auton();
    //trapDoor.extend();
}




void opcontrol()
{
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
                outtake(9000);
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

        if (throttle < 5) {
            chassis.arcade(throttle, (int)low_power_steer_curve(steer), true);
        } else {
            leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            chassis.curvature(throttle, steer, false);
        }

        pros::delay(10);
    }
}