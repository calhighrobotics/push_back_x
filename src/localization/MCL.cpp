#include "MCL.h"
#include "globals.h" 
#include <algorithm>
#include <cmath>

namespace MCL {

    // --- Tunable Defaults --
    double PARAMS_ROT_NOISE_STD = 0.0751;    
    double PARAMS_TRANS_BASE = 0.05;      
    double PARAMS_TRANS_GAIN = 0.02;      
    double PARAMS_SENSOR_SIGMA = 0.8; 

    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0; 
    double map_min_x = -72.0, map_max_x = 72.0, map_min_y = -72.0, map_max_y = 72.0;
    
    pros::Mutex particle_mutex;
    std::mt19937 Random(std::random_device{}());

    // --- Sensor Constructor with Trig Caching ---
    MCLDistanceSensor::MCLDistanceSensor(pros::Distance sensor_, double localX, double localY, double angleDeg) 
        : Sensor(sensor_), LocalX(localX), LocalY(localY), AngleDeg(angleDeg) {
        measurement = -1;
        cos_off = std::cos(angleDeg * M_PI / 180.0);
        sin_off = std::sin(angleDeg * M_PI / 180.0);
    }

    void MCLDistanceSensor::Measure() {
        int dist_mm = Sensor.get_distance();
        if (dist_mm > 10 && dist_mm < (SENSOR_MAX_RANGE_IN * 25.4)) { 
            measurement = dist_mm * 0.0393701; 
        } else {
            measurement = -1.0;
        }
    }

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

    std::vector<MCLDistanceSensor> Sensors = {
        MCLDistanceSensor(frontDistance, -3.0,  10.5,   0),  
        MCLDistanceSensor(leftDistance,  -6.4, -0.5,   -90), 
        MCLDistanceSensor(rightDistance,  6.3, -0.5,    90), 
        MCLDistanceSensor(backDistance,  -3.0, -10.5,  180), 
    };

    std::vector<Particle> particles(NUM_PARTICLES);
    std::vector<Particle> resample_buffer(NUM_PARTICLES); 

    // --- Helpers ---
    inline double degToRad(double deg) { return deg * M_PI / 180.0; }
    inline double radToDeg(double rad) { return rad * 180.0 / M_PI; }

    // 🔥 FIX 1: Mathematically Robust Wrap for all inputs
    inline double wrapAngleDeg(double angle) {
        angle = std::fmod(angle + 180.0, 360.0);
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    double gaussian_sample(double mean, double stddev) {
        std::normal_distribution<double> dist(mean, stddev);
        return dist(Random);
    }

    // 🔥 Optimization: Raycasting with Trig Addition Formulas (No new Sin/Cos calls)
    double getRaycastDistance(const Particle& p, const MCLDistanceSensor& sensor) {
        double sensX = p.x + (sensor.LocalY * p.sin_t + sensor.LocalX * p.cos_t);
        double sensY = p.y + (sensor.LocalY * p.cos_t - sensor.LocalX * p.sin_t);
        
        // Ray direction using: sin(A+B) = sinA cosB + cosA sinB
        //                      cos(A+B) = cosA cosB - sinA sinB
        double dx = p.sin_t * sensor.cos_off + p.cos_t * sensor.sin_off;
        double dy = p.cos_t * sensor.cos_off - p.sin_t * sensor.sin_off;

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
            
            if (u >= 0.0 && u <= 1.0 && t > 0.0) {
                if (t < minDist) minDist = t;
            }
        }
        return (minDist > SENSOR_MAX_RANGE_IN) ? -1.0 : minDist;
    }

    void StartMCL(double x, double y, double theta) {
        particle_mutex.take();
        
        // 🔥 FIX 2: Correct infinity-based reset for bounds accumulation
        map_min_x = map_min_y = std::numeric_limits<double>::infinity();
        map_max_x = map_max_y = -std::numeric_limits<double>::infinity();

        for (int i = 0; i < MAP_SEGMENT_COUNT; ++i) {
            map_min_x = std::min({map_min_x, MAP_SEGMENTS[i].x0, MAP_SEGMENTS[i].x1});
            map_max_x = std::max({map_max_x, MAP_SEGMENTS[i].x0, MAP_SEGMENTS[i].x1});
            map_min_y = std::min({map_min_y, MAP_SEGMENTS[i].y0, MAP_SEGMENTS[i].y1});
            map_max_y = std::max({map_max_y, MAP_SEGMENTS[i].y0, MAP_SEGMENTS[i].y1});
        }

        for (auto& p : particles) {
            p.x = gaussian_sample(x, 2.0);
            p.y = gaussian_sample(y, 2.0);
            p.theta = wrapAngleDeg(gaussian_sample(theta, 5.0));
            p.weight = 1.0 / NUM_PARTICLES;
            p.cos_t = std::cos(degToRad(p.theta));
            p.sin_t = std::sin(degToRad(p.theta));
        }
        particle_mutex.give();
    }

    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        
        while (true) {
            lemlib::Pose currOdom = chassis.getPose();
            double dX_global = currOdom.x - prevOdom.x;
            double dY_global = currOdom.y - prevOdom.y;
            double dTheta = wrapAngleDeg(currOdom.theta - prevOdom.theta);

            if (std::abs(dTheta) > 45.0) dTheta = 0.0; 

            double cosP = std::cos(degToRad(prevOdom.theta));
            double sinP = std::sin(degToRad(prevOdom.theta));
            double dF = dX_global * sinP + dY_global * cosP;
            double dS = dX_global * cosP - dY_global * sinP;

            particle_mutex.take();

            for (auto& p : particles) {
                double transStd = PARAMS_TRANS_BASE + (PARAMS_TRANS_GAIN * std::abs(dTheta)) + (0.05 * std::hypot(dF, dS));
                double dx_l = dF + gaussian_sample(0, transStd);
                double dy_l = dS + gaussian_sample(0, transStd);

                p.x += dx_l * p.sin_t + dy_l * p.cos_t;
                p.y += dx_l * p.cos_t - dy_l * p.sin_t;
                
                // 🔥 Optimization: Boundary clamping to keep particles in-bounds
                p.x = std::clamp(p.x, map_min_x, map_max_x);
                p.y = std::clamp(p.y, map_min_y, map_max_y);
                
                p.theta = wrapAngleDeg(p.theta + dTheta + gaussian_sample(0, PARAMS_ROT_NOISE_STD));
                
                double rad = degToRad(p.theta);
                p.cos_t = std::cos(rad);
                p.sin_t = std::sin(rad);
            }
            prevOdom = currOdom;

            std::vector<MCLDistanceSensor*> active;
            for (auto& s : Sensors) { s.Measure(); if (s.measurement > 0) active.push_back(&s); }

            if (!active.empty()) {
                double max_log = -1e9;
                std::vector<double> logs(NUM_PARTICLES);

                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    double logL = 0.0;
                    for (auto* s : active) {
                        double pred = getRaycastDistance(particles[i], *s);
                        double err = (pred < 0) ? (s->measurement - SENSOR_MAX_RANGE_IN) : (s->measurement - pred);
                        logL += -(err * err) / (2 * PARAMS_SENSOR_SIGMA * PARAMS_SENSOR_SIGMA);
                    }
                    logs[i] = logL;
                    if (logL > max_log) max_log = logL;
                }

                double sumW = 0;
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    particles[i].weight = std::exp(logs[i] - max_log);
                    sumW += particles[i].weight;
                }
                for (auto& p : particles) p.weight /= (sumW > 1e-9 ? sumW : 1.0);
            }

            double mX = 0, mY = 0, sS = 0, sC = 0, neff_s = 0;
            for (const auto& p : particles) {
                mX += p.x * p.weight; mY += p.y * p.weight;
                sS += p.sin_t * p.weight; sC += p.cos_t * p.weight;
                neff_s += p.weight * p.weight;
            }
            global_X = mX; global_Y = mY; global_Theta = radToDeg(std::atan2(sS, sC));
            double neff = (neff_s > 0) ? (1.0 / neff_s) : 0;
            global_Confidence = std::clamp(neff / NUM_PARTICLES, 0.0, 1.0);

            // 🔥 FIX 3: Selective Resampling (Threshold-based)
            if (global_Confidence < RESAMPLE_THRESHOLD && (!active.empty() || global_Confidence < 0.15)) {
                double injectP = 0.02 + (1.0 - global_Confidence) * 0.15;
                int keep = NUM_PARTICLES * (1.0 - injectP);
                double r = std::uniform_real_distribution<double>(0, 1.0 / keep)(Random), c = particles[0].weight;
                int idx = 0;
                
                for (int i = 0; i < keep; ++i) {
                    double U = r + (double)i / keep;
                    while (U > c && idx < NUM_PARTICLES - 1) c += particles[++idx].weight;
                    resample_buffer[i] = particles[idx];
                    resample_buffer[i].weight = 1.0 / NUM_PARTICLES;
                }
                
                std::uniform_real_distribution<double> fX(map_min_x, map_max_x), fY(map_min_y, map_max_y);
                for (int i = keep; i < NUM_PARTICLES; ++i) {
                    double randTheta = wrapAngleDeg(currOdom.theta + gaussian_sample(0, 15));
                    resample_buffer[i] = {fX(Random), fY(Random), randTheta, 1.0/NUM_PARTICLES, std::cos(degToRad(randTheta)), std::sin(degToRad(randTheta))};
                }
                particles = resample_buffer;
            }

            particle_mutex.give();
            pros::delay(20);
        }
    }
}