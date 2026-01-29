#pragma once
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <random>

// Holds our 5 tuning parameters
struct TuningConfig {
    float q_x, q_y, q_theta, r_ang, r_vel;
};

class EvolutionTuner {
private:
    TuningConfig parent;
    TuningConfig child;
    double parent_score;
    
    // Global step size (sigma)
    // In Log-Space, this represents percentage change.
    // 0.1 = ~10% variation, 0.5 = ~50% variation.
    float sigma; 
    
    // 1/5th Rule Counters
    int success_count = 0;
    int cycles = 0;
    const int ADAPTATION_RATE = 5; 

    // Sensitivity Scales (Multipliers for sigma)
    // Q parameters need free movement.
    // R parameters should generally be stiffer to prevent instability.
    struct Scales { float q, r; } scales = { 1.0f, 0.8f };

    std::mt19937 rng;
    std::normal_distribution<float> dist;

    bool first_run = true;
    
    // Constraints
    const float MAX_GAIN = 200000.0f;
    const float MIN_GAIN = 100.0f;
    const float MAX_DAMP = 5000.0f;
    const float MIN_DAMP = 0.1f;

public:
    TuningConfig pending_config;
    bool is_finished = false;

    EvolutionTuner(TuningConfig start_guess) : dist(0.0, 1.0) {
        parent = start_guess;
        parent_score = 1e9; // Infinity
        
        // Initial "Search Radius"
        // 0.2 means we start by varying parameters by approx +/- 20%
        sigma = 0.2f; 
        
        rng.seed(pros::millis());
    }

    // Helper: Reflective Clamping
    // Prevents "sticking" to the wall by bouncing back
    float reflect(float val, float min, float max) {
        if (val < min) return min + (min - val);
        if (val > max) return max - (val - max);
        return val;
    }

    TuningConfig clampConfig(TuningConfig c) {
        c.q_x = reflect(c.q_x, MIN_GAIN, MAX_GAIN);
        c.q_y = reflect(c.q_y, MIN_GAIN, MAX_GAIN);
        c.q_theta = reflect(c.q_theta, MIN_GAIN, MAX_GAIN);
        c.r_ang = reflect(c.r_ang, MIN_DAMP, MAX_DAMP);
        c.r_vel = reflect(c.r_vel, MIN_DAMP, MAX_DAMP);
        return c;
    }

    void next(double last_score) {
        // CASE 1: First run
        if (first_run) {
            parent_score = last_score;
            first_run = false;
            mutate();
            return;
        }

        // CASE 2: Evaluate Child
        // Lower score is better
        if (last_score < parent_score) {
            parent = child; // Evolution successful!
            parent_score = last_score;
            success_count++;
        } 
        // Else: Child died. Parent remains.

        // CASE 3: Adapt Step Size (1/5th Rule)
        cycles++;
        if (cycles >= ADAPTATION_RATE) {
            double success_rate = (double)success_count / cycles;
            
            // If success > 20%, we are effectively just hill-climbing. Accelerate!
            if (success_rate > 0.20) sigma *= 1.2f; 
            
            // If success < 20%, we are lost in noise. Shrink step size.
            else if (success_rate < 0.20) sigma *= 0.82f;
            
            success_count = 0;
            cycles = 0;
            
            // Safety Clamp on Sigma (Don't let it explode or vanish)
            if(sigma > 1.0f) sigma = 1.0f; // Max 100% mutation
            if(sigma < 0.001f) sigma = 0.001f; // Min 1% mutation
        }

        mutate();
    }

    // LOG-SPACE MUTATION WITH COUPLING
    void mutate() {
    child = parent;
    
    // CHANGE 1: Independent Mutation (Decoupling)
    // We remove "noise_q_common" so X and Y can find their own perfect values.
    
    // Evolve Gains (Q)
    child.q_x     *= std::exp(dist(rng) * sigma * scales.q);
    child.q_y     *= std::exp(dist(rng) * sigma * scales.q); // Now independent
    child.q_theta *= std::exp(dist(rng) * sigma * scales.q);
    
    // Evolve Damping/Penalties (R)
    // CHANGE 2: Increase R sensitivity
    // If the robot is jerky, we need R to adapt FAST to calm it down.
    // We use 'scales.q' here effectively to give it equal playing field.
    child.r_ang   *= std::exp(dist(rng) * sigma * scales.q); 
    child.r_vel   *= std::exp(dist(rng) * sigma * scales.q);

    child = clampConfig(child);
    pending_config = child;
}

    TuningConfig getBestConfig() { return parent; }
    double getBestScore() { return parent_score; }
    float getSigma() { return sigma; }
};