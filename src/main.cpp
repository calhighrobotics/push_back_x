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
    // 1. Hardware & Warning Initialization
    chassis.calibrate();
    temp_warning();
    motor_disconnect_warning();
    distance_sensor_disconnect_warning();
    create_alliance_selector();
    
    // 2. Initial Pose Definition
    // Adjust these coordinates to your actual starting tile
    double start_x = -51.25;
    double start_y = -18.5;
    double start_theta = 180.0;

    // 3. Start MCL Subsystem
    chassis.setPose(start_x, start_y, start_theta); 
    //MCL::StartMCL(start_x, start_y, start_theta);
    
    // Launch MCL as a background task (20ms loop internally)
    //pros::Task mcl_task(MCL::MonteCarlo);

    // 4. Console & UI Focus
    console.focus();

    // 5. Comparison Monitoring Task
    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
            
            // Get standard LemLib Odometry Pose
            lemlib::Pose odomPose = chassis.getPose();
            
            // --- LEMLIB ODOMETRY ---
            console.printf("X: %.2f  Y: %.2f\n", odomPose.x, odomPose.y);
            console.printf("Theta: %.2f\n\n", odomPose.theta);
            
            // --- MCL LOCALIZATION ---
            /*
            console.printf("X: %.2f  Y: %.2f\n", MCL::global_X, MCL::global_Y);
            console.printf("Theta: %.2f\n", MCL::global_Theta);
            
            // Display our new Confidence Metric (N_eff based)
            console.printf("Conf: %.1f%%\n", MCL::global_Confidence * 100.0);

            // --- ERROR DELTA ---
            double error_dist = std::hypot(odomPose.x - MCL::global_X, odomPose.y - MCL::global_Y);
            console.printf("Delta Dist: %.2f in\n", error_dist);
            */
            pros::delay(50); // Lower refresh rate for the screen to prevent flickering
        }
    });
}

void disabled() {
    console.focus();
}

void competition_initialize() {
    selector.focus();
}


void distanceCalibration() {

    std::cout << "\n==== DISTANCE SENSOR CALIBRATION START ====\n";

    // IMPORTANT:
    // Start robot flush against wall manually.
    // Then this backs up known distance.

    const double backUpInches = 5.0;

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -backUpInches, 2000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(500);

    std::vector<pros::Distance*> sensors = {
        &frontDistance,
        &leftDistance,
        &backDistance,
        &rightDistance,
        &frontDistance2
    };

    std::vector<std::string> names = {
        "Front",
        "Left",
        "Back",
        "Right",
        "Front 2"
    };

    std::vector<int> sweepAngles = {0, 30, 45};
    const int samplesPerAngle = 5;

    for (int s = 0; s < sensors.size(); s++) {

        std::cout << "\n--- Calibrating " << names[s] << " Sensor ---\n";

        int baseHeading = s * 90;

        chassis.turnToHeading(baseHeading, 1500);
        chassis.waitUntilDone();
        pros::delay(500);

        for (int angle : sweepAngles) {

            int target = baseHeading + angle;

            chassis.turnToHeading(target, 1000);
            chassis.waitUntilDone();
            pros::delay(400);

            double sum = 0;
            int validCount = 0;

            for (int i = 0; i < samplesPerAngle; i++) {

                int reading = sensors[s]->get_distance();

                if (reading > 30 && reading < 5000) {
                    sum += reading;
                    validCount++;
                }

                pros::delay(40);
            }

            if (validCount == 0) {
                std::cout << names[s]
                          << " @ +" << angle
                          << " deg: INVALID\n";
                continue;
            }

            double avg_mm = sum / validCount;
            double avg_in = avg_mm / 25.4;

            std::cout << names[s]
                      << " @ +" << angle
                      << " deg: "
                      << avg_mm << " mm ("
                      << avg_in << " in)"
                      << std::endl;
        }

        chassis.turnToHeading(baseHeading, 1000);
        chassis.waitUntilDone();
        pros::delay(500);
    }

    std::cout << "\n==== CALIBRATION COMPLETE ====\n";
}

void find_tracking_center(float turnVoltage, uint32_t time_ms) {
    chassis.setPose(0, 0, 0);
    std::vector<float> xs, ys, thetas;
    uint32_t start = pros::millis();
    while (pros::millis() - start < time_ms) {
        leftMotors.move_voltage(turnVoltage * 1000);
        rightMotors.move_voltage(-turnVoltage * 1000);

        auto pose = chassis.getPose(false);  


        xs.push_back(pose.x);
        ys.push_back(pose.y);
        thetas.push_back(pose.theta);

        pros::delay(20);
    }

    leftMotors.brake();
    rightMotors.brake();

    std::cout << "X_0 = [";
    for (size_t i = 0; i < xs.size(); i++) {
        std::cout << "(" << xs[i] << "," << ys[i] << ")";
        if (i + 1 < xs.size()) std::cout << ",";
        pros::delay(7); 
    }
    std::cout << "]\n";

    pros::delay(50);

    std::cout << "θ_t = [";
    for (size_t i = 0; i < thetas.size(); i++) {
        std::cout << thetas[i];
        if (i + 1 < thetas.size()) std::cout << ",";
        pros::delay(7);
    }
    std::cout << "]\n";
}


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

void collect_velocity_vs_voltage_data(bool turning = false) {
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
        rightMotors.move_voltage(direction * input * 1000 * turning ? -1 : 1);

        pros::delay(1000);
        float v_sum = 0;
        int n;
        for (n = 0; n < 500; ++n) {
            v_sum += (std::fabs(leftMotors.get_actual_velocity()*0.0032429f) + std::fabs(rightMotors.get_actual_velocity()*0.0032429f)) / 2;
            pros::delay(10);
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

void collect_voltage_step_data(float step_input, unsigned int duration, bool turning = false) {
    std::vector<float> outputs = {};
    duration *= 100;
    outputs.reserve(duration);

    leftMotors.move_voltage(step_input * 1000);
    rightMotors.move_voltage(step_input * 1000 * turning ? -1 : 1);

    for (int i = 0; i < duration; ++i) {
        auto speed = (std::fabs(leftMotors.get_actual_velocity()*0.0032429f) + std::fabs(rightMotors.get_actual_velocity()*0.0032429f)) / 2;
        std::cout << Vector2((float)i / 100, speed).latex() << "," << std::flush;
        pros::delay(10);
    }

    leftMotors.brake();
    rightMotors.brake();
    std::cout << "\b" << std::endl;
}





void autonomous() {
    //distanceCalibration();
    //find_tracking_center(5, 6000);
    //collect_velocity_vs_voltage_data(false);
    //collect_voltage_step_data(4000, 4, false);
    //awp_auton();
    elim_auton();
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