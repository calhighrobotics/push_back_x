#include "main.h"
#include "globals.h" 
#include "liblvgl/core/lv_obj_pos.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "auton/autonRoutines.h"
#include "robodash/views/selector.hpp"
#include "colorSort.h"
#include "warnings.h"
#include <string>
#include "MCL.h"
#include "visionAlignment.h"
#include "auton/autonFunctions.h"

#include "ltv.h"
#include "paths.h"

void alliance_btn_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    lv_obj_t * label = lv_obj_get_child(btn, 0);

    if(code == LV_EVENT_CLICKED) {
        if(allianceColor == RED) {
            allianceColor = Color::BLUE;
            lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), 0);
            lv_label_set_text(label, "BLUE");
        } else {
            allianceColor = Color::RED;
            lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);
            lv_label_set_text(label, "RED");
        }

        
        std::cout << "Alliance switched to: " << (allianceColor == Color::RED ? "Red" : "Blue") << std::endl;
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
    temp_warning();
    motor_disconnect_warning();
    distance_sensor_disconnect_warning();
    color_sensor.set_led_pwm(100);
    /*
   chassis.setPose(-63.5, -18.5, 180);
   MCL::StartMCL(-63.5, -18.5, 180);
   pros::Task mclTask(MCL::MonteCarlo);
    */

    /*
    chassis.setPose(-48.5, -54.56, 270);
    console.focus();
    
    */
    //calibrate_vision();
    create_alliance_selector();
    console.focus();
    pros::Task screen_task([&]() {
        lemlib::Pose pose{0,0,0};
        while (true) {
            console.clear();
            pose = chassis.getPose();


            //distancePose dpose = distanceReset(false);
            console.printf("X: %f\n", pose.x);
            console.printf("Y: %f\n", pose.y);
            console.printf("Theta: %f\n", pose.theta);
            
            /*
            console.printf("D X: %f\n", dpose.x);
            console.printf("D Y: %f\n", dpose.y);
            console.printf("Using X: %d\n", dpose.using_odom_x);
            console.printf("Using Y: %d\n", dpose.using_odom_y);
            */
            
            
            //console.printf("X MCL: %f\n", MCL::X);
            //console.printf("Y MCL: %f\n", MCL::Y);
            
            pros::delay(100);
        }
    });
}

void disabled() {
    selector.focus();
}

void competition_initialize() {
}

void distanceCalibration() {
    // Reset and back up
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -4, 2000, {.forwards = false});
    chassis.waitUntilDone();

    std::vector<pros::Distance*> sensors = {
        &frontDistance, &leftDistance, &backDistance, &rightDistance
    };
    std::vector<std::string> names = {
        "Front", "Left", "Back", "Right"
    };

    // Sweep angles relative to sensor facing
    std::vector<int> sweepAngles = {0, 30, 45};

    for (int s = 0; s < sensors.size(); s++) {
        int baseHeading = s * 90;

        // Face the sensor toward the wall
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

            // Return to sensor zero between sweeps
            chassis.turnToHeading(baseHeading, 700);
            chassis.waitUntilDone();
            pros::delay(400);
        }
    }
}

void autonomous() {
    //selector.run_auton();
    //skills_auton();
    //awp_auton();
    //find_tracking_center(5, 5000);
    distanceCalibration();
}


void opcontrol() {
    std::cout << allianceColor << std::endl;
    bool trapDoor_commanded = false;
    while(true)
    {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
        {
            midgoal_first = true;
        }

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
            score_longgoal(12000, allianceColor);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            score_midgoal();
            ramp_up_time += 10;
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
        {
            intake_to_basket();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            if(descore.is_extended())
                descore.retract();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
        {
            throttle = 0;
            steer = 0;
            leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            leftMotors.brake();
            rightMotors.brake();
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            if(!trapDoor_commanded)
            {
                trapDoor_commanded = true;
                trapDoor.extend();
            }
            else {
                trapDoor_commanded = false;
            }
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            matchload.toggle();
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            color_sort_enable = !color_sort_enable;
            controller.rumble(".");
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            alignToGoal(1);
        }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            lowGoalAligner.toggle();
        }
        else
        {
            ramp_up_time = 0;
            resting_state(trapDoor_commanded);
            midgoal_first = false;
        }

        if(throttle < 5)
            chassis.arcade(throttle, steer, false);
        else
            chassis.curvature(throttle, steer, false);
        pros::delay(10);
    }
}