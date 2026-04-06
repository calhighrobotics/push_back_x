#ifndef MCL_H
#define MCL_H

#include "api.h"        
#include <vector>

namespace MCL {
    constexpr int NUM_PARTICLES = 800; 
    constexpr float RESAMPLE_THRESHOLD = 0.4f;   
    constexpr float SENSOR_MAX_RANGE_IN = 55.0f; 
    extern double PARAMS_TRANS_BASE;      
    extern double PARAMS_TRANS_GAIN;      

    extern double global_X;
    extern double global_Y;
    extern double global_Theta;

    struct MCLDistanceSensor {
        pros::Distance Sensor;
        double LocalX;
        double LocalY;
        double AngleDeg;
        
        double measurement;
        double cos_off;
        double sin_off;

        MCLDistanceSensor(pros::Distance sensor_, double localX, double localY, double angleDeg);
        
        void Measure();
    };

    extern std::vector<MCLDistanceSensor> Sensors;
    void StartMCL(double x, double y);

    void MonteCarlo();

} 

#endif 