#include "globals.h"
#include "pros/ai_vision.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

bool followMultipleObjectsPID(std::vector<int> code_ids, float target_distance_in, int timeout_ms = 3000) {
    float kP_turn = 50.0f; 
    float kD_turn = 100.0f; 
    
    float kP_drive = 400.0f; 
    float kD_drive = 800.0f; 

    const int CENTER_X = 160; 
    const float ERROR_THRESHOLD_X = 10.0f; 
    const float ERROR_THRESHOLD_DIST = 1.5f; 
    const int FRAMES_TO_SETTLE = 12; 
    
    const float DISTANCE_CONSTANT = 400.0f; 

    int aligned_counter = 0;
    int lost_frame_counter = 0;
    bool is_aligned = false;
    bool first_frame = true; 

    float prev_error_x = 0;
    float prev_error_dist = 0;
    float filtered_area = 0.0f; 
    
    int start_time = pros::millis();

    while ((pros::millis() - start_time) < timeout_ms) {
        
        std::vector<pros::AIVision::Object> objects = ai_vision.get_all_objects(); 
        
        pros::AIVision::Object best_goal;
        float best_area = 0.0f;
        bool found_valid_target = false;

        for (const auto& obj : objects) {

            // ✅ Correct type for your PROS version
            if (pros::AIVision::is_type(obj, pros::AivisionDetectType::code)) { 
                
                bool matches_requested_code =
                    std::find(code_ids.begin(), code_ids.end(), obj.id) != code_ids.end();
                
                if (matches_requested_code) {

                    float width = obj.object.element.width;
                    float height = obj.object.element.height;

                    if (width > 15 && height > 15) {

                        float current_area = width * height;
                        
                        if (current_area > best_area) {
                            best_area = current_area;
                            best_goal = obj;
                            found_valid_target = true;
                        }
                    }
                }
            }
        }

        float turn_voltage = 0; 
        float drive_voltage = 0; 

        if (found_valid_target) { 
            lost_frame_counter = 0;
            
            if (first_frame) {
                filtered_area = best_area; 
            } else {
                filtered_area = (filtered_area * 0.7f) + (best_area * 0.3f); 
            }

            // ✅ FIXED: use xoffset (position), NOT width
            float error_x = best_goal.object.element.xoffset - CENTER_X;
            
            float current_distance_in =
                DISTANCE_CONSTANT / std::sqrt(std::max(filtered_area, 1.0f)); 
            
            float error_dist = current_distance_in - target_distance_in; 

            float derivative_x = 0;
            float derivative_dist = 0;
            
            if (!first_frame) {
                derivative_x = error_x - prev_error_x;
                derivative_dist = error_dist - prev_error_dist;
            }

            first_frame = false; 

            prev_error_x = error_x;
            prev_error_dist = error_dist;

            bool heading_settled = std::abs(error_x) <= ERROR_THRESHOLD_X;
            bool distance_settled = std::abs(error_dist) <= ERROR_THRESHOLD_DIST;

            if (heading_settled && distance_settled) {
                aligned_counter++;
                turn_voltage = 0;
                drive_voltage = 0;
                
                if (aligned_counter > FRAMES_TO_SETTLE) {
                    is_aligned = true;
                    break; 
                }
            } else {
                aligned_counter = 0;
                
                turn_voltage = (error_x * kP_turn) + (derivative_x * kD_turn);
                drive_voltage = (error_dist * kP_drive) + (derivative_dist * kD_drive);

                if (!heading_settled) {
                    drive_voltage = 0;
                }
            }

        } else {
            lost_frame_counter++;

            if (lost_frame_counter > 3) {
                turn_voltage = 0; 
                drive_voltage = 0;
                aligned_counter = 0;
                first_frame = true; 
            }
        }

        float left_out = std::clamp(drive_voltage + turn_voltage, -12000.0f, 12000.0f);
        float right_out = std::clamp(drive_voltage - turn_voltage, -12000.0f, 12000.0f);

        leftMotors.move_voltage(left_out);
        rightMotors.move_voltage(right_out);

        pros::delay(10); 
    }

    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
    
    return is_aligned;
}