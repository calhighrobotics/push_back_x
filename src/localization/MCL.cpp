#include <new> // <--- REQUIRED for GCC 14+ Placement New
#include "MCL.h"
#include "globals.h" 
#include <algorithm>
#include <limits>
#include <cmath>
#include <vector>

using namespace std;

namespace MCL {

    // --- Tunable Defaults --

    double PARAMS_ROT_NOISE_STD = 0.0751;    
    double PARAMS_TRANS_BASE = 0.05;      
    double PARAMS_TRANS_GAIN = 0.02;      
    double PARAMS_SENSOR_SIGMA = 1.2;     

    // --- Global State ---
    double global_X = 0;
    double global_Y = 0;
    double global_Theta = 0;
    double global_Confidence = 0; 
    
    pros::Mutex particle_mutex;
    std::mt19937 Random(std::random_device{}());

    // --- Map Data ---
    static const Segment MAP_SEGMENTS[] = {
        {72, 72, 72, -72}, {72, -72, -72, -72}, {-72, -72, -72, 72}, {-72, 72, 72, 72},
        {20.9, 46.25, 24.4, 46.25}, {24.4, 46.25, 24.4, 49.75}, {24.4, 49.75, 20.9, 49.75}, {20.9, 49.75, 20.9, 46.25},
        {-24.4, 46.25, -20.9, 46.25}, {-20.9, 46.25, -20.9, 49.75}, {-20.9, 49.75, -24.4, 49.75}, {-24.4, 49.75, -24.4, 46.25},
        {20.9, -49.75, 24.4, -49.75}, {24.4, -49.75, 24.4, -46.25}, {24.4, -46.25, 20.9, -46.25}, {20.9, -46.25, 20.9, -49.75},
        {-24.4, -49.75, -20.9, -49.75}, {-20.9, -49.75, -20.9, -46.25}, {-20.9, -46.25, -24.4, -46.25}, {-24.4, -46.25, -24.4, -49.75},
        {-72, 45.5, -67, 45.5}, {-67, 45.5, -67, 50.5}, {-67, 50.5, -72, 50.5}, {-72, 50.5, -72, 45.5},
        {67, 45.5, 72, 45.5}, {72, 45.5, 72, 50.5}, {72, 50.5, 67, 50.5}, {67, 50.5, 67, 45.5},
        {-72, -50.5, -67, -50.5}, {-67, -50.5, -67, -45.5}, {-67, -45.5, -72, -45.5}, {-72, -45.5, -72, -50.5},
        {67, -50.5, 72, -50.5}, {72, -50.5, 72, -45.5}, {72, -45.5, 67, -45.5}, {67, -45.5, 67, -50.5},
    };
    static const int MAP_SEGMENT_COUNT = sizeof(MAP_SEGMENTS) / sizeof(Segment);


    // Initialize sensors (Using the names from your globals.h)
    std::vector<MCLDistanceSensor> Sensors = {
        MCLDistanceSensor(frontDistance, -3, -0.75, 0),
        MCLDistanceSensor(leftDistance, -6.4, -0.5, 90),
        MCLDistanceSensor(rightDistance, 6.3, -0.5, -90),
        MCLDistanceSensor(backDistance, -3, -10.5, 180),
    };

    std::vector<Particle> particles(NUM_PARTICLES);
    std::vector<Particle> resample_buffer; 

    // --- Helpers ---
    inline double degToRad(double deg) { return deg * M_PI / 180.0; }
    inline double radToDeg(double rad) { return rad * 180.0 / M_PI; }
    inline double wrapAngleDeg(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }
    double gaussian_sample(double mean, double stddev) {
        std::normal_distribution<double> dist(mean, stddev);
        return dist(Random);
    }
    bool isConfident() { return global_Confidence > 0.7; }

    // --- Raycasting ---
    double getRaycastDistance(double px, double py, double pthetaDeg, const MCLDistanceSensor& sensor) {
        double pThetaRad = degToRad(pthetaDeg);
        double c = std::cos(pThetaRad);
        double s = std::sin(pThetaRad);
        double sensX = px + (sensor.LocalX * c - sensor.LocalY * s);
        double sensY = py + (sensor.LocalX * s + sensor.LocalY * c);
        double rayAngleRad = degToRad(pthetaDeg + sensor.AngleDeg);
        double dx = std::cos(rayAngleRad);
        double dy = std::sin(rayAngleRad);

        double minDist = 10000.0; 
        for (int i = 0; i < MAP_SEGMENT_COUNT; ++i) {
            const Segment& seg = MAP_SEGMENTS[i];
            double r_px = seg.x0 - sensX;
            double r_py = seg.y0 - sensY;
            double s_px = seg.x1 - seg.x0;
            double s_py = seg.y1 - seg.y0;
            double r_cross_s = dx * s_py - dy * s_px; 
            if (std::abs(r_cross_s) < 1e-6) continue;
            double t = (r_px * s_py - r_py * s_px) / r_cross_s;
            double u = (r_px * dy - r_py * dx) / r_cross_s;
            if (u >= 0.0 && u <= 1.0 && t > 0.0) if (t < minDist) minDist = t;
        }
        if (minDist > SENSOR_MAX_RANGE_IN) return -1.0;
        return minDist;
    }

    // --- Main Loop ---
    void StartMCL(double x, double y, double theta) {
        particle_mutex.take();
        global_X = x; global_Y = y; global_Theta = theta;
        resample_buffer.resize(NUM_PARTICLES);
        for (auto& p : particles) {
            p.x = gaussian_sample(x, 2.0);
            p.y = gaussian_sample(y, 2.0);
            p.theta = wrapAngleDeg(gaussian_sample(theta, 5.0));
            p.weight = 1.0 / NUM_PARTICLES;
        }
        particle_mutex.give();
    }

    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        pros::delay(20);

        while (true) {
            lemlib::Pose currOdom = chassis.getPose();
            double dX_global = currOdom.x - prevOdom.x;
            double dY_global = currOdom.y - prevOdom.y;
            double dTheta = wrapAngleDeg(currOdom.theta - prevOdom.theta);

            double prevRad = degToRad(prevOdom.theta);
            double cosP = std::cos(prevRad);
            double sinP = std::sin(prevRad);
            double dForward = dX_global * sinP + dY_global * cosP;
            double dStrafe  = dX_global * cosP - dY_global * sinP;

            particle_mutex.take();

            // 1. Prediction (Motion)
            for (auto& p : particles) {
                double pRad = degToRad(p.theta);
                double transStd = PARAMS_TRANS_BASE + PARAMS_TRANS_GAIN * std::abs(dTheta);
                
                if(transStd > 2.0) transStd = 2.0; 

                double dx_local = dForward + gaussian_sample(0, transStd);
                double dy_local = dStrafe  + gaussian_sample(0, transStd);

                p.x += dx_local * std::sin(pRad) + dy_local * std::cos(pRad);
                p.y += dx_local * std::cos(pRad) - dy_local * std::sin(pRad);
                p.theta = wrapAngleDeg(p.theta + dTheta + gaussian_sample(0, PARAMS_ROT_NOISE_STD));
            }
            prevOdom = currOdom;

            // 2. Update (Sensors)
            std::vector<MCLDistanceSensor*> activeSensors;
            for (auto& sensor : Sensors) {
                sensor.Measure();
                if (sensor.measurement > 0) activeSensors.push_back(&sensor);
            }

            double max_log_weight = -1e9;
            std::vector<double> log_weights(NUM_PARTICLES);
            bool sensors_active = !activeSensors.empty();

            if (sensors_active) {
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    double logLikelihood = 0.0;
                    for (auto* sensor : activeSensors) {
                        double predicted = getRaycastDistance(particles[i].x, particles[i].y, particles[i].theta, *sensor);
                        double error;
                        double sigma = PARAMS_SENSOR_SIGMA; 

                        if (predicted < 0) {
                            error = 12.0; 
                        } else {
                            error = sensor->measurement - predicted;
                        }
                        logLikelihood += -(error * error) / (2 * sigma * sigma);
                    }
                    log_weights[i] = logLikelihood;
                    if (logLikelihood > max_log_weight) max_log_weight = logLikelihood;
                }
                double totalWeight = 0;
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    particles[i].weight = std::exp(log_weights[i] - max_log_weight);
                    totalWeight += particles[i].weight;
                }
                if (totalWeight > 1e-9) {
                    for (auto& p : particles) p.weight /= totalWeight;
                } else {
                    for (auto& p : particles) p.weight = 1.0 / NUM_PARTICLES;
                }
            }

            // 3. Estimation & Confidence
            double meanX = 0, meanY = 0, sumSin = 0, sumCos = 0;
            double varX = 0, varY = 0;

            for (const auto& p : particles) {
                meanX += p.x * p.weight;
                meanY += p.y * p.weight;
                double rad = degToRad(p.theta);
                sumSin += std::sin(rad) * p.weight;
                sumCos += std::cos(rad) * p.weight;
            }
            
            for (const auto& p : particles) {
                varX += p.weight * (p.x - meanX) * (p.x - meanX);
                varY += p.weight * (p.y - meanY) * (p.y - meanY);
            }
            
            double stdDev = std::sqrt(varX + varY);
            double conf = 1.0 - (stdDev / 10.0); 
            if(conf < 0) conf = 0;
            if(conf > 1) conf = 1;
            
            global_Confidence = conf;
            global_X = meanX;
            global_Y = meanY;
            global_Theta = radToDeg(std::atan2(sumSin, sumCos));

            // 4. Resampling
            if (sensors_active) {
                std::uniform_real_distribution<double> dist(0.0, 1.0 / NUM_PARTICLES);
                double r = dist(Random);
                double c = particles[0].weight;
                int idx = 0;
                int particles_to_keep = NUM_PARTICLES * (1.0 - RAND_INJECTION_PERCENT);
                
                for (int i = 0; i < particles_to_keep; ++i) {
                    double U = r + (double)i / NUM_PARTICLES;
                    while (U > c && idx < NUM_PARTICLES - 1) {
                        idx++;
                        c += particles[idx].weight;
                    }
                    resample_buffer[i] = particles[idx];
                    resample_buffer[i].weight = 1.0 / NUM_PARTICLES;
                }
                std::uniform_real_distribution<double> fieldDist(-70, 70); 
                for (int i = particles_to_keep; i < NUM_PARTICLES; ++i) {
                    resample_buffer[i].x = fieldDist(Random);
                    resample_buffer[i].y = fieldDist(Random);
                    resample_buffer[i].theta = wrapAngleDeg(currOdom.theta + gaussian_sample(0, 10.0));
                    resample_buffer[i].weight = 1.0 / NUM_PARTICLES;
                }
                particles = resample_buffer;
            }

            particle_mutex.give();
            pros::delay(20);
        }
    }
}