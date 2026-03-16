#include "main.h"
#include "globals.h" 
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/core/lv_obj_pos.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/vision.hpp"
#include "robodash/views/selector.hpp"
#include <string>
#include "MCL.h"
#include "distanceReset.h"
#include <atomic> 
#include <memory> 

rd::Console console;
void initialize() {
    chassis.calibrate();
    console.focus();
    
    double start_x = 0;
    double start_y = -24.0;
    double start_theta = 270.0;

    chassis.setPose(start_x, start_y, start_theta); 
    MCL::StartMCL(start_x, start_y);
    pros::Task mcl_task(MCL::MonteCarlo);
    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
            
            lemlib::Pose odomPose = chassis.getPose();
            
            console.printf("[ RAW ODOMETRY ]\n");
            console.printf("X: %.2f  Y: %.2f\n", odomPose.x, odomPose.y);
            console.printf("Theta: %.2f\n\n", odomPose.theta);
            
            console.printf("[ MCL ESTIMATE ]\n");
            console.printf("X: %.2f  Y: %.2f\n", MCL::global_X, MCL::global_Y);
            
            pros::delay(50);
        }
    });
}

void disabled() {
    
}

void competition_initialize() {
}

#include "main.h"
#include <vector>
#include <cmath>
#include <limits>
#include <string>

extern lemlib::Chassis chassis;

struct Reading {
    double heading;
    double distance;
};

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
void AutoTuneSensorOffsets(pros::Distance& sensor, std::string sensor_name, double known_angle_deg) {
    printf("\n=========================================\n");
    printf("🔍 STARTING CALIBRATION: %s\n", sensor_name.c_str());
    printf("=========================================\n");

    // Reset Odometry/IMU heading to 0
    chassis.setPose(0, 0, 0);
    pros::delay(200);

    std::vector<Reading> data;
    chassis.tank(60, -60); 

    uint32_t start_time = pros::millis();
    double start_heading = chassis.getPose().theta;
    double current_heading = start_heading;
    
    double min_dist = 9999.0;
    double angle_at_min = 0.0;

    printf("Spinning 90 degrees and collecting data...\n");

    while (std::abs(current_heading - start_heading) < 90 && (pros::millis() - start_time < 4000)) {
        current_heading = chassis.getPose().theta;
        int dist_mm = sensor.get_distance();
        
        if (dist_mm > 10 && dist_mm < 1016) { 
            double d_inches = dist_mm * 0.0393701;
            data.push_back({current_heading, d_inches});

            if (d_inches < min_dist) {
                min_dist = d_inches;
                angle_at_min = current_heading;
            }
        }
        pros::delay(20); 
    }
    
    chassis.tank(0, 0); 

    if (data.size() < 15) {
        printf("❌ FAILED: Not enough valid data points collected.\n");
        return;
    }

    printf("Data collected. Running grid search solver for X and Y...\n");

    // We lock the angle to what you explicitly passed into the function
    double alpha_rad = deg2rad(known_angle_deg);

    double best_X = 0, best_Y = 0, best_W = 0;
    double lowest_error = std::numeric_limits<double>::infinity();

    // Iterate through a grid of possible X and Y offsets at 0.1 inch resolution
    for (double test_x = -10.0; test_x <= 10.0; test_x += 0.1) {
        for (double test_y = -10.0; test_y <= 10.0; test_y += 0.1) {
            
            // Where the wall is relative to the start
            double test_W = min_dist + (test_x * std::cos(deg2rad(angle_at_min)) - test_y * std::sin(deg2rad(angle_at_min)));
            
            double total_error = 0;
            int valid_points = 0;
            
            for (const auto& pt : data) {
                double theta_rad = deg2rad(pt.heading);
                double denominator = std::cos(theta_rad + alpha_rad);
                
                if (std::abs(denominator) < 0.15) continue; 
                
                double expected_d = (test_W - (test_x * std::cos(theta_rad) - test_y * std::sin(theta_rad))) / denominator;
                
                double error = pt.distance - expected_d;
                total_error += (error * error); 
                valid_points++;
            }

            if (valid_points > 0 && total_error < lowest_error) {
                lowest_error = total_error;
                best_X = test_x;
                best_Y = test_y;
                best_W = test_W;
            }
        }
    }

    double avg_error = std::sqrt(lowest_error / data.size());

    printf("\n🎯 SOLVER RESULTS FOR: %s\n", sensor_name.c_str());
    printf("-----------------------------------------\n");
    printf("Local X       :  %.2f inches\n", best_X);
    printf("Local Y       :  %.2f inches\n", best_Y);
    printf("Angle Offset  :  %.2f degrees (Locked)\n\n", known_angle_deg);
    
    printf("--- Diagnostic Info ---\n");
    printf("Average Error :  %.3f inches\n", avg_error);
    printf("=========================================\n\n");
}

void autonomous() {
    //AutoTuneSensorOffsets(frontDistance, "Front Distance", 0);
    /*
    chassis.setPose(-24, -24, 90);
    chassis.moveToPoint(24, -24, 10000);
    chassis.turnToPoint(24, 24, 10000);
    chassis.moveToPoint(24, 24, 10000);
    chassis.turnToPoint(-24, 24, 10000);
    chassis.moveToPoint(-24, 24, 10000);
    chassis.turnToPoint(-24, -24, 10000);
    chassis.moveToPoint(-24, -24, 10000);
    */

    
    chassis.setPose(0, -24, 270);
    chassis.moveToPose(-36, 0, 0, 10000, {});
    chassis.moveToPose(0, 36, 90, 10000, {});
    chassis.moveToPose(36, 0, 180, 10000);
    chassis.moveToPose(0, -36, 270, 10000);
    
}
void opcontrol()
{

    while (true) {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    
        chassis.curvature(throttle, steer, false);

        pros::delay(10);
    }
}