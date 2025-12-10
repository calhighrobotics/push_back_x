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

Color current_color = Color::RED;
lv_obj_t *btn = nullptr;

void btn_click(lv_event_t * e) {
    current_color = (current_color == Color::RED) ? Color::BLUE : Color::RED;

    lv_color_t lv_col = (current_color == Color::RED)
                        ? lv_palette_main(LV_PALETTE_RED)
                        : lv_palette_main(LV_PALETTE_BLUE);

    lv_obj_set_style_bg_color(lv_event_get_target(e), lv_col, LV_PART_MAIN);
}

void create_button() {
    btn = lv_btn_create(lv_scr_act());

    lv_obj_set_size(btn, 100, 50);
    lv_obj_set_pos(btn, 100, 60);

    lv_obj_add_event_cb(btn, btn_click, LV_EVENT_CLICKED, NULL);

    lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
}

RamsetePathFollower ramsete(config, 2, 0.7);

void initialize() {
    chassis.calibrate();
    
    pros::Task screen_task([&]() {
        lemlib::Pose pose{0,0,0};
        temp_warning();
        motor_disconnect_warning();
        distance_sensor_disconnect_warning();
        create_button();
        MCL::StartMCL();
        pros::Task mclTask(MCL::MonteCarlo);
        std::vector<std::string> paths = {test_path, right_1};
        ramsete.precompute_paths(paths);
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
    selector.focus();
}

void competition_initialize() {
}

void autonomous() {
    colorSort(current_color);
    ramsete.followPath(right_1, {.path_index = 1});
    distanceReset();
    selector.run_auton();
}

void opcontrol() {
    while(true)
    {
        float throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        float steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

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



