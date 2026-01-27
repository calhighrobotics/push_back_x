#pragma once
#include "ltv.h"
#include <vector>
#include <iostream>

class TwiddleTuner {
public:
    // Pointers to the weights we want to optimize
    // 0: q_x/q_y (Position), 1: q_theta (Heading), 2: r_vel (Damping), 3: r_ang (Steering)
    std::vector<double> params;
    std::vector<double> d_params; // The "Step Size" for each param
    
    int param_idx = 0; // Which param are we tuning right now?
    int state = 0;     // 0: Init, 1: Check +, 2: Check -
    double best_err = 100000000.0;
    
    TwiddleTuner() {
        // INITIAL GUESS (Current Best)
        params = {28000.0, 8000.0, 15.0, 80.0}; 
        
        // STEP SIZES (How much to tweak each time)
        // Aggressive for Q, careful for R
        d_params = {2000.0, 1000.0, 5.0, 5.0};
    }

    // Call this before running the path to get the experimental config
    LTVPathFollower::ltvConfig getConfig() {
        LTVPathFollower::ltvConfig cfg;
        
        // --- 1. APPLY PARAMS ---
        cfg.q_x = params[0];
        cfg.q_y = params[0]; // Lock X/Y (Isotropic)
        cfg.q_theta = params[1];
        cfg.r_vel = params[2];
        cfg.r_ang = params[3];

        // --- 2. CRITICAL SAFETY CLAMPS ---
        // Prevent Division by Zero or Instability
        
        if (cfg.q_x < 100.0) cfg.q_x = 100.0; 
        if (cfg.q_theta < 100.0) cfg.q_theta = 100.0;

        // R (Damping) must never be zero! 
        if (cfg.r_vel < 10.0) cfg.r_vel = 10.0;
        if (cfg.r_ang < 10.0) cfg.r_ang = 10.0;
        
        // Upper bounds (Safety)
        if (cfg.r_vel > 200.0) cfg.r_vel = 200.0; 
        if (cfg.r_ang > 200.0) cfg.r_ang = 200.0; 

        // --- 3. STANDARD SETTINGS ---
        cfg.max_lin_correction = 999;
        cfg.max_ang_correction = 999;
        cfg.test = true; // Snap to start pose for consistency
        return cfg;
    }

    // Call this AFTER the robot finishes the path
    void update(double score) {
        std::cout << "[TUNER] Param: " << param_idx << " | Score: " << score << " | Best: " << best_err << std::endl;

        if (state == 0) {
            // First run ever
            best_err = score;
            state = 1;
            params[param_idx] += d_params[param_idx];
            return;
        }

        if (state == 1) {
            // We just tried increasing the weight. Did it work?
            if (score < best_err) {
                // YES: It got better!
                best_err = score;
                d_params[param_idx] *= 1.1; // Increase step size
                
                // Move to next parameter
                param_idx = (param_idx + 1) % params.size();
                params[param_idx] += d_params[param_idx];
                state = 1;
            } else {
                // NO: It got worse. Try decreasing instead.
                params[param_idx] -= 2 * d_params[param_idx]; // Go the other way
                state = 2;
            }
        } else if (state == 2) {
            // We just tried decreasing the weight. Did it work?
            if (score < best_err) {
                // YES: Decreasing was the right move.
                best_err = score;
                d_params[param_idx] *= 1.1;
            } else {
                // NO: Increasing failed, Decreasing failed.
                // Reset to original and make step size smaller (fine tune)
                params[param_idx] += d_params[param_idx];
                d_params[param_idx] *= 0.9;
            }
            
            // Move to next parameter
            param_idx = (param_idx + 1) % params.size();
            params[param_idx] += d_params[param_idx];
            state = 1;
        }
        
        // Print Recommendation
        std::cout << "--- NEW RECOMMENDED WEIGHTS ---" << std::endl;
        std::cout << "Q_POS: " << params[0] << std::endl;
        std::cout << "Q_ANG: " << params[1] << std::endl;
        std::cout << "R_VEL: " << params[2] << std::endl;
        std::cout << "R_ANG: " << params[3] << std::endl;
        std::cout << "-------------------------------" << std::endl;
    }
};