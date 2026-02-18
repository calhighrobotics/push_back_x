#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include <vector>
#include <cmath>
#include <random>

namespace MCL {

    // --- Configuration ---
    constexpr int NUM_PARTICLES = 500; 
    constexpr double SENSOR_MAX_RANGE_IN = 60.0; 
    constexpr double RAND_INJECTION_PERCENT = 0.03; 

    // --- Tunable Parameters ---
    extern double PARAMS_ROT_NOISE_STD;   
    extern double PARAMS_TRANS_BASE;      
    extern double PARAMS_TRANS_GAIN;      
    extern double PARAMS_SENSOR_SIGMA;    

    struct Particle {
        double x, y, theta, weight;
    };

    struct Segment {
        double x0, y0, x1, y1;
    };

    class MCLDistanceSensor {
    public:
        MCLDistanceSensor(pros::Distance sensor_, double localX, double localY, double angleDeg) 
            : Sensor(sensor_), LocalX(localX), LocalY(localY), AngleDeg(angleDeg) {
            measurement = -1;
        }
        void Measure() {
            int dist_mm = Sensor.get_distance();
            if (dist_mm > 10 && dist_mm < (SENSOR_MAX_RANGE_IN * 25.4)) { 
                measurement = dist_mm * 0.0393701; 
            } else {
                measurement = -1.0;
            }
        }
        pros::Distance Sensor;
        double measurement, LocalX, LocalY, AngleDeg; 
    };

    // Global Access
    extern double global_X;
    extern double global_Y;
    extern double global_Theta;
    extern double global_Confidence; // 0.0 (Lost) to 1.0 (Locked)
    
    extern std::vector<MCLDistanceSensor> Sensors;

    void StartMCL(double x, double y, double theta);
    void MonteCarlo();
    
    // New Helper: Should we override Odom?
    bool isConfident(); 
}