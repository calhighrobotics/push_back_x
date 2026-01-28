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
    std::cout << std::endl;
    for (auto &s : logs2) std::cout << s, pros::delay(50);
}

void disabled() {
    selector.focus();
}

void competition_initialize() {
}

void distanceCalibration()
{
    // 1. Reset position and move back 4 inches
    chassis.setPose(0, 0, 0);
    // Move to y = -4. forwards = false ensures we reverse to that point.
    chassis.moveToPoint(0, -4, 2000, {.forwards = false});
    chassis.waitUntilDone(); // Wait for the move to finish before turning

    // 2. Setup sensors and targets
    std::vector<pros::Distance*> sensors = {&frontDistance, &leftDistance, &backDistance, &rightDistance};
    std::vector<std::string> sensorNames = {"Front", "Left", "Back", "Right"};
    std::vector<int> angles = {0, 30, 45};

    // 3. Loop through the target angles
    for (int angle : angles)
    {
        // Turn to the specific heading (0, then 30, then 45)
        chassis.turnToHeading(angle, 2000);
        chassis.waitUntilDone(); // Wait for the turn to finish
        
        // Optional: Short delay to let the sensor readings stabilize after moving
        pros::delay(200); 

        // 4. Log measurements from all sensors for this angle
        printf("--- Calibration at %d Degrees ---\n", angle);
        for(int i = 0; i < sensors.size(); i++)
        {
            int reading = sensors.at(i)->get();
            // Prints: "Front Sensor: 120 mm"
            printf("%s Sensor: %d mm\n", sensorNames.at(i).c_str(), reading);
        }
    }
}

void autonomous() {
    //selector.run_auton();
    //skills_auton();
    //awp_auton();
    //find_tracking_center(5, 5000);
    elim_auton();
}


#include "main.h"
#include "EvolutionTuner.hpp"
// ... [Your other includes] ...

void opcontrol() {
    
    // 1. Robot Physical Config (Keep your existing values)
    const VelocityControllerConfig velConfig{
        5.8432642308, 0.213526937516, 1.14429410811, 
        0.906356177095, 0.347072436421, 11.4953431776, 54.5797495382
    };

    LTVPathFollower follower(velConfig);

    // 2. Initial Seed (The "Corner King" Tune)
    TuningConfig start_weights = { 
        60000.0, // q_x
        60000.0, // q_y
        25000.0, // q_theta
        130.0,   // r_ang
        15.0     // r_vel
    };

    // Initialize Log-Space Evolution Tuner
    EvolutionTuner tuner(start_weights);
    
    double last_score = 0;
    bool is_tuning = true;
    int gen_count = 0;

    controller.clear();
    pros::delay(50);
    controller.print(0, 0, "Press A to Evolve");

    while (true) {
        
        // --- EVOLUTION STEP ---
        if (controller.get_digital_new_press(DIGITAL_A) && is_tuning) {
            
            // 1. First Run: Establish Baseline
            if (gen_count == 0) {
                controller.print(0, 0, "Baseline Run...");
                
                LTVPathFollower::ltvConfig ltv_cfg;
                ltv_cfg.q_x = start_weights.q_x;
                ltv_cfg.q_y = start_weights.q_y;
                ltv_cfg.q_theta = start_weights.q_theta;
                ltv_cfg.r_ang = start_weights.r_ang;
                ltv_cfg.r_vel = start_weights.r_vel;
                ltv_cfg.test = true;
                ltv_cfg.log = false;

                LTVPathFollower::PathScore result = follower.followPath(awp_1, ltv_cfg);
                last_score = result.final_score;
                
                tuner.next(last_score); // Feed baseline to tuner
                gen_count++;
            }
            // 2. Evolution Runs
            else {
                tuner.next(last_score); // Generate Child based on parent success/fail
                TuningConfig next_cfg = tuner.pending_config;

                LTVPathFollower::ltvConfig ltv_cfg;
                ltv_cfg.q_x = next_cfg.q_x;
                ltv_cfg.q_y = next_cfg.q_y;
                ltv_cfg.q_theta = next_cfg.q_theta;
                ltv_cfg.r_ang = next_cfg.r_ang;
                ltv_cfg.r_vel = next_cfg.r_vel;
                ltv_cfg.test = true;
                ltv_cfg.log = false;

                // Logging for Terminal
                std::cout << "\n=== GEN " << gen_count << " (Sig: " << tuner.getSigma() << ") ===" << std::endl;
                std::cout << "Try: Qx " << (int)next_cfg.q_x << " | Qt " << (int)next_cfg.q_theta 
                          << " | Ra " << (int)next_cfg.r_ang << std::endl;

                // Controller UI
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "G:%d Sig:%.2f", gen_count, tuner.getSigma());
                controller.print(1, 0, "Best: %.0f", tuner.getBestScore());

                // Run Path
                LTVPathFollower::PathScore result = follower.followPath(tuning_curve_2, ltv_cfg);
                last_score = result.final_score;
                gen_count++;
                
                // Feedback
                bool improved = (last_score < tuner.getBestScore());
                std::cout << "SCORE: " << last_score << (improved ? " [NEW BEST!]" : " [FAIL]") << std::endl;
                
                if (improved) controller.rumble("-"); // Long rumble = Improvement
                else controller.rumble(".");          // Short rumble = Fail
            }
        }

        // --- SAVE AND QUIT ---
        if (controller.get_digital_new_press(DIGITAL_B)) {
            TuningConfig best = tuner.getBestConfig();
            std::cout << "\n****** OPTIMIZED WEIGHTS ******" << std::endl;
            std::cout << "float q_x = " << best.q_x << ";" << std::endl;
            std::cout << "float q_y = " << best.q_y << ";" << std::endl;
            std::cout << "float q_theta = " << best.q_theta << ";" << std::endl;
            std::cout << "float r_ang = " << best.r_ang << ";" << std::endl;
            std::cout << "float r_vel = " << best.r_vel << ";" << std::endl;
            std::cout << "*******************************" << std::endl;

            controller.clear();
            pros::delay(50);
            controller.print(0, 0, "Saved to Term.");
            is_tuning = false;
        }

        pros::delay(20);
    }
}