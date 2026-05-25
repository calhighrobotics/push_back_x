// Copyright 2026 California High Robotics, Team 1516X
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "main.h"
#include "lemlib/api.hpp"
#include <vector>
#include <cmath>
#include <numeric>

// Configuration for a single calibration step
struct CalibrationConfig {
    std::string name;           // e.g., "Front", "Left"
    pros::Distance* sensor;     // Pointer to the PROS sensor
    double base_heading;        // Heading where this sensor faces the wall (e.g., 0 for Front, 90 for Right)
};

struct CalibrationResult {
    std::string name;
    double offset_in;
    double confidence_score; // Mean Squared Error (closer to 0 is better)
};

class SensorCalibrator {
private:
    lemlib::Chassis* chassis;
    pros::Imu* imu;

    // Linear Regression to solve: Y = D - x * X
    // Where Y = dist * cos(theta), X = sin(theta)
    CalibrationResult solveLeastSquares(const std::string& name, const std::vector<double>& theta_rads, const std::vector<double>& dists_in) {
        double n = theta_rads.size();
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;

        for (size_t i = 0; i < n; i++) {
            // Apply the geometric correction for angled walls
            double x = sin(theta_rads[i]);
            double y = dists_in[i] * cos(theta_rads[i]); 

            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_xx += x * x;
        }

        // Calculate slope (m) and intercept (b)
        // Slope m = -offset
        double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        double intercept = (sum_y - slope * sum_x) / n;

        double calculated_offset = -slope;

        // Calculate Residual Error (Confidence)
        double total_error = 0;
        for (size_t i = 0; i < n; i++) {
            double x = sin(theta_rads[i]);
            double y_actual = dists_in[i] * cos(theta_rads[i]);
            double y_pred = slope * x + intercept;
            total_error += pow(y_actual - y_pred, 2);
        }

        return { name, calculated_offset, total_error / n };
    }

public:
    SensorCalibrator(lemlib::Chassis* chassis_ptr, pros::Imu* imu_ptr) 
        : chassis(chassis_ptr), imu(imu_ptr) {}

    // Main routine to calibrate a list of sensors
    std::vector<CalibrationResult> calibrateAll(const std::vector<CalibrationConfig>& configs) {
        std::vector<CalibrationResult> results;
        
        // The requested angles relative to the sensor's center
        // Note: 35 degrees is steep for some lasers, ensure wall is non-reflective tape if possible
        std::vector<double> relative_angles = {0, 30, 35}; 

        printf("--- STARTING SENSOR CALIBRATION ---\n");

        for (const auto& config : configs) {
            printf("Calibrating %s Sensor...\n", config.name.c_str());
            
            std::vector<double> recorded_thetas;
            std::vector<double> recorded_dists;

            // 1. Rotate to face the wall (base_heading)
            chassis->turnToHeading(config.base_heading, 1000);
            pros::delay(500);

            // 2. Sweep through the test angles
            for (double rel_angle : relative_angles) {
                // Calculate target heading
                double target = config.base_heading + rel_angle;
                
                // Turn using LemLib
                chassis->turnToHeading(target, 1000, {.maxSpeed = 60});
                
                // Wait for settle (very important for accurate distance)
                pros::delay(600); 

                // 3. Data Collection
                // Get true angle from IMU (converted to radians for math)
                // We subtract base_heading to get local theta relative to the wall normal
                double current_heading = imu->get_heading();
                
                // Handle 0-360 wrap for math
                double diff = current_heading - config.base_heading;
                if (diff > 180) diff -= 360;
                if (diff < -180) diff += 360;
                
                double theta_rad = diff * M_PI / 180.0;
                
                // Get distance in inches (convert mm to in)
                double d_mm = config.sensor->get();
                if (d_mm > 9990) {
                     printf("  [ERR] %s sensor saw infinity/error at %.1f deg\n", config.name.c_str(), diff);
                     continue; // Skip bad readings
                }
                double d_in = d_mm / 25.4;

                recorded_thetas.push_back(theta_rad);
                recorded_dists.push_back(d_in);

                printf("  Angle: %.1f | Dist: %.3f\n", diff, d_in);
            }

            // 4. Solve
            if (recorded_dists.size() >= 2) {
                results.push_back(solveLeastSquares(config.name, recorded_thetas, recorded_dists));
            } else {
                printf("  [FAIL] Not enough valid points for %s\n", config.name.c_str());
                results.push_back({config.name, 0.0, 999.9});
            }
        }

        // Return to 0
        chassis->turnToHeading(0, 1000);
        return results;
    }
};