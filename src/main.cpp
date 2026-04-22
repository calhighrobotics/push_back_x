#include "main.h"
#include "globals.h" 
#include "liblvgl/core/lv_obj_pos.h"
#include "auton/autonRoutines.h"
#include "pros/misc.hpp"
#include "robodash/views/image.hpp"
#include "robodash/views/selector.hpp"
#include "colorSort.h"
#include "warnings.h"
#include <cstdint>
#include <string>
#include "MCL.h"
#include "distanceReset.h"
#include "auton/autonFunctions.h"
#include "motorDashboard.h"
#include "IntakeAntiJam.h"

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
    {"Right 7 Split", right_auton_split},
    {"Right 7 Wing", right_7_wing},
    {"Right Rush", right_rush},
    {"Left 7 Block", left_auton_split},
    {"Left Rush", left_rush},
    {"AWP", awp_auton},
    {"Skills", skills_auton},
    {"Carry", carry_auton}
});

rd::Console console;
rd::Image image = rd::Image(&callogo, "logo");

void auton_check(std::optional<rd::Selector::routine_t> auton)
{
    if(selector.get_auton().value().name == "Skills")
    {
        intake_lift.extend();
    }
    //precompute_auton_paths(selector.get_auton().value().name);
}

void initialize() {
    chassis.calibrate();
    motor_disconnect_warning();
    distance_sensor_disconnect_warning();
    create_alliance_selector();
    init_motor_dashboard();
    pros::Task jamTask(antiJamTask);
    selector.on_select(auton_check);

    //Delete if things start going wrong
    //vertical_tracking_sensor.set_data_rate(5);
    //horizontal_tracking_sensor.set_data_rate(5);

    /*
    chassis.setPose(-48, -12, 90);
    distancePose pose = distanceReset(true);
    double start_x = pose.x;
    double start_y = pose.y;
    double start_theta = 90;
    chassis.setPose(start_x, start_y, start_theta); 
    */
    chassis.setPose(27, 48, 90);

    //MCL::StartMCL(start_x, start_y);

    //pros::Task mcl_task(MCL::MonteCarlo);
    selector.focus();
    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
            
            lemlib::Pose odomPose = chassis.getPose();
            //distancePose pose = distanceReset(false);
            console.printf("X: %.3f  Y: %.3f\n", odomPose.x, odomPose.y);
            console.printf("Theta: %.3f\n\n", odomPose.theta);
            //console.printf("Collided? %s\n", chassis.detect_collision() ? "YES" : "NO");
            //console.printf("Color: %s\n", get_color() == Color::RED ? "RED" : "BLUE");
            //distancePose pose = distanceReset(false, false);
            //console.printf("DSR X: %.3f  DSR Y: %.3f\n", pose.x, pose.y);
            //console.printf("DSR using Odom X: %s  using Odom Y: %s\n", pose.using_odom_x ? "true" : "false", pose.using_odom_y ? "true" : "false");
            
            //console.printf("X: %.2f  Y: %.2f\n", MCL::global_X, MCL::global_Y);
            //console.printf("Theta: %.2f\n", MCL::global_Theta);
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

//Min turn: Sensitvity at low speed
//Max Turn: Sensitivity at high speed
float dynamic_steer_curve(float throttle, float steer, float min_turn_scale = 0.4, float max_turn_scale = 1.5) {
    float t = std::abs(throttle) / 127.0f;
    float s = steer / 127.0f;
    float curve = t * t; 
    float scale = min_turn_scale + (max_turn_scale - min_turn_scale) * curve;
    float s_out = s * scale;
    return s_out * 127.0f;
}

void opcontrol() {
    image.focus();
    float time = 0;
    midgoal_first = false;
    bool trapDoor_commanded = false;
    bool matchload_on = true;
    float maxDeltaThrottle = 4;
    float prevThrottle = 0;
    float starting_pitch = imu.get_roll();
    float roll_forward_threshold = 1;
    float roll_backward_threshold = -1;
    bool slewOn = false;
    
    int radio_cooldown = 50;     
    bool needs_clear = false;
    bool needs_print = false;    
    bool needs_rumble = false;   

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    enum class ScoringMode {
        NONE, INTAKE, OUTTAKE, MIDGOAL, LONGGOAL, MANUAL_UP, BRAKE
    };

    jamManager.enable_anti_jam(antiJamEnabled);

    while (true) {

        float throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        float steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        if (std::abs(throttle) < 5) throttle = 0;
        if (std::abs(steer) < 5) steer = 0;

        if(slewOn) {
            bool isTipping = (imu.get_roll() - starting_pitch > roll_forward_threshold || imu.get_roll() - starting_pitch < roll_backward_threshold);
            bool isSlowingDown = (std::abs(throttle) < std::abs(prevThrottle));
            bool isReversing = (prevThrottle > 0 && throttle < 0) || (prevThrottle < 0 && throttle > 0);

            if (isTipping && (isSlowingDown || isReversing)) {
                throttle = lemlib::slew(throttle, prevThrottle, maxDeltaThrottle);
            }
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            antiJamEnabled = !antiJamEnabled;
            jamManager.enable_anti_jam(antiJamEnabled);
            
            needs_clear = true;
            needs_print = true; 
            needs_rumble = true;
        }

        if (radio_cooldown >= 50) {
            if (needs_clear) {
                controller.clear_line(0);
                needs_clear = false;
                radio_cooldown = 0;
            }
            else if (needs_print) {
                controller.print(0, 0, "ANTIJAM %s", (antiJamEnabled) ? "ON" : "OFF");
                needs_print = false;
                radio_cooldown = 0;   
            } 
            else if (needs_rumble) {
                controller.rumble("."); 
                needs_rumble = false;
                radio_cooldown = 0;   
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            if(matchloader.is_extended()) {
                matchloader.retract();
            } else {
                matchloader.extend();
            }
        }
        
        ScoringMode scoringMode = ScoringMode::NONE;
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            mid_descore.extend();
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

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            descore.retract(); 
            scoringMode = ScoringMode::BRAKE;
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
                descore.extend();
                mid_descore.retract();
                break;
            case ScoringMode::BRAKE:
                break;
        }

        chassis.curvature(throttle,dynamic_steer_curve(throttle, steer, 0.4, 1.5), false);
        if(liveReplay)
        {
            float leftVel = leftMotors.get_actual_velocity() * 0.00324173f;
            float rightVel = rightMotors.get_actual_velocity() * 0.00324173f;
            std::cout  << time << ", " << (leftVel + rightVel) / 2 << std::endl;
        }
        prevThrottle = throttle;   
        radio_cooldown += 10;   
        
        if(liveReplay)
        {
            pros::delay(20);
            time += 0.02;
        }
        else {
            pros::delay(10);
        }
    }
}