#pragma once
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <random>

struct TuningConfig {
    float q_x, q_y, q_theta, r_ang, r_vel;
};

class EvolutionTuner {
private:
    TuningConfig parent;
    TuningConfig child;
    double parent_score;
    float sigma; 
    int success_count = 0;
    int cycles = 0;
    const int ADAPTATION_RATE = 5; 
    struct Scales { float q, r; } scales = { 1.0f, 0.8f };
    std::mt19937 rng;
    std::normal_distribution<float> dist;
    bool first_run = true;
    const float MAX_GAIN = 200000.0f;
    const float MIN_GAIN = 100.0f;
    const float MAX_DAMP = 5000.0f;
    const float MIN_DAMP = 0.1f;

public:
    TuningConfig pending_config;
    bool is_finished = false;

    EvolutionTuner(TuningConfig start_guess) : dist(0.0, 1.0) {
        parent = start_guess;
        parent_score = 1e9;
        sigma = 0.2f; 
        rng.seed(pros::millis());
    }

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
        if (first_run) {
            parent_score = last_score;
            first_run = false;
            mutate();
            return;
        }
        if (last_score < parent_score) {
            parent = child;
            parent_score = last_score;
            success_count++;
        }
        cycles++;
        if (cycles >= ADAPTATION_RATE) {
            double success_rate = (double)success_count / cycles;
            if (success_rate > 0.20) sigma *= 1.2f; 
            else if (success_rate < 0.20) sigma *= 0.82f;
            success_count = 0;
            cycles = 0;
            if(sigma > 1.0f) sigma = 1.0f;
            if(sigma < 0.001f) sigma = 0.001f;
        }
        mutate();
    }

    void mutate() {
        child = parent;
        child.q_x     *= std::exp(dist(rng) * sigma * scales.q);
        child.q_y     *= std::exp(dist(rng) * sigma * scales.q);
        child.q_theta *= std::exp(dist(rng) * sigma * scales.q);
        child.r_ang   *= std::exp(dist(rng) * sigma * scales.q); 
        child.r_vel   *= std::exp(dist(rng) * sigma * scales.q);
        child = clampConfig(child);
        pending_config = child;
    }

    TuningConfig getBestConfig() { return parent; }
    double getBestScore() { return parent_score; }
    float getSigma() { return sigma; }
};
