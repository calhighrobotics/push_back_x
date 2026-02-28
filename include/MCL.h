#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include <vector>
#include <cmath>
#include <random>
#include <limits>

namespace MCL {

    // --- Configuration ---
    constexpr int NUM_PARTICLES = 500; 
    constexpr double SENSOR_MAX_RANGE_IN = 60.0; 
    constexpr double RESAMPLE_THRESHOLD = 0.5; // Resample when Neff < 50%

    // --- Tunable Parameters ---
    extern double PARAMS_ROT_NOISE_STD;   
    extern double PARAMS_TRANS_BASE;      
    extern double PARAMS_TRANS_GAIN;      
    extern double PARAMS_SENSOR_SIGMA;    

    struct Particle {
        double x, y, theta, weight;
        double cos_t, sin_t; 
    };

    struct Segment {
        double x0, y0, x1, y1;
    };

    class MCLDistanceSensor {
    public:
        MCLDistanceSensor(pros::Distance sensor_, double localX, double localY, double angleDeg);
        
        void Measure();
        
        pros::Distance Sensor;
        double measurement, LocalX, LocalY, AngleDeg; 
        double cos_off, sin_off; // Precomputed sensor offsets
    };

    // --- Global Access ---
    extern double global_X;
    extern double global_Y;
    extern double global_Theta;
    extern double global_Confidence; 
    
    extern std::vector<MCLDistanceSensor> Sensors;

    void StartMCL(double x, double y, double theta);
    void MonteCarlo();
    bool isConfident(); 
}