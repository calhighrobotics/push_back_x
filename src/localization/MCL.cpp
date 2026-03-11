#include "MCL.h"
#include "globals.h" 
#include <algorithm>
#include <cmath>
#include <vector>

namespace MCL {

    double PARAMS_TRANS_BASE = 0.2;      
    double PARAMS_TRANS_GAIN = 0.02;      
    double PARAMS_SENSOR_SIGMA = 2; 
    double PARAMS_REL_SIGMA = 5.0; 

    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0; 
    double map_min_x = -72.0, map_max_x = 72.0, map_min_y = -72.0, map_max_y = 72.0;
    
    pros::Mutex particle_mutex;
    std::mt19937 Random(std::random_device{}());

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

    struct OptSegment { double x0, y0, x1, y1, dx, dy, nx, ny; };
    OptSegment opt_segments[MAP_SEGMENT_COUNT];

    struct AABB { double min_x, max_x, min_y, max_y; };
    AABB map_aabbs[MAP_SEGMENT_COUNT];

    std::vector<MCLDistanceSensor> Sensors = {
        MCLDistanceSensor(frontDistance, -3.0, -0.75, 0),  
        MCLDistanceSensor(leftDistance, 0.5, -7.2, 90), 
        MCLDistanceSensor(rightDistance, -0.5, 6.3, -90), 
        MCLDistanceSensor(backDistance, 3.0, -10.5, 180), 
    };

    std::vector<Particle> particles(NUM_PARTICLES);
    std::vector<Particle> resample_buffer(NUM_PARTICLES); 

    struct ValidMeas { double pred; double meas; double weight; };
    struct RayHit { double dist; double weight; };

    inline double degToRad(double deg) { return deg * M_PI / 180.0; }
    inline double radToDeg(double rad) { return rad * 180.0 / M_PI; }
    inline double wrapAngleDeg(double angle) {
        angle = std::fmod(angle + 180.0, 360.0);
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    double gaussian_sample(double mean, double stddev) {
        std::normal_distribution<double> dist(mean, stddev);
        return dist(Random);
    }

    RayHit getRaycastDistance(double sensX, double sensY, double dx, double dy) {
        double minDist = 10000.0; 
        double bestWeight = 1.0; 

        double ray_end_x = sensX + dx * SENSOR_MAX_RANGE_IN;
        double ray_end_y = sensY + dy * SENSOR_MAX_RANGE_IN;
        double r_min_x = std::min(sensX, ray_end_x);
        double r_max_x = std::max(sensX, ray_end_x);
        double r_min_y = std::min(sensY, ray_end_y);
        double r_max_y = std::max(sensY, ray_end_y);

        for (int i = 0; i < MAP_SEGMENT_COUNT; ++i) {
            const AABB& box = map_aabbs[i];
            if (r_max_x < box.min_x || r_min_x > box.max_x || 
                r_max_y < box.min_y || r_min_y > box.max_y) continue;

            const OptSegment& seg = opt_segments[i];
            double r_px = seg.x0 - sensX;
            double r_py = seg.y0 - sensY;
            double r_cross_s = dx * seg.dy - dy * seg.dx; 
            
            if (std::abs(r_cross_s) < 1e-4) continue; 
            
            double t = (r_px * seg.dy - r_py * seg.dx) / r_cross_s;
            double u = (r_px * dy - r_py * dx) / r_cross_s;
            
            if (u >= 0.0 && u <= 1.0 && t > 0.0) {
                if (t < minDist) {
                    minDist = t;
                    bestWeight = 0.5 + 0.5 * std::abs(dx * seg.nx + dy * seg.ny);
                }
            }
        }
        
        if (minDist <= 0 || minDist > SENSOR_MAX_RANGE_IN) return {-1.0, 1.0};
        return {minDist, bestWeight};
    }

    void StartMCL(double x, double y) {
        particle_mutex.take();
        map_min_x = map_min_y = std::numeric_limits<double>::infinity();
        map_max_x = map_max_y = -std::numeric_limits<double>::infinity();

        for (int i = 0; i < MAP_SEGMENT_COUNT; ++i) {
            map_min_x = std::min({map_min_x, MAP_SEGMENTS[i].x0, MAP_SEGMENTS[i].x1});
            map_max_x = std::max({map_max_x, MAP_SEGMENTS[i].x0, MAP_SEGMENTS[i].x1});
            map_min_y = std::min({map_min_y, MAP_SEGMENTS[i].y0, MAP_SEGMENTS[i].y1});
            map_max_y = std::max({map_max_y, MAP_SEGMENTS[i].y0, MAP_SEGMENTS[i].y1});
            
            map_aabbs[i] = {
                std::min(MAP_SEGMENTS[i].x0, MAP_SEGMENTS[i].x1),
                std::max(MAP_SEGMENTS[i].x0, MAP_SEGMENTS[i].x1),
                std::min(MAP_SEGMENTS[i].y0, MAP_SEGMENTS[i].y1),
                std::max(MAP_SEGMENTS[i].y0, MAP_SEGMENTS[i].y1)
            };

            opt_segments[i].x0 = MAP_SEGMENTS[i].x0;
            opt_segments[i].y0 = MAP_SEGMENTS[i].y0;
            opt_segments[i].x1 = MAP_SEGMENTS[i].x1;
            opt_segments[i].y1 = MAP_SEGMENTS[i].y1;
            opt_segments[i].dx = MAP_SEGMENTS[i].x1 - MAP_SEGMENTS[i].x0;
            opt_segments[i].dy = MAP_SEGMENTS[i].y1 - MAP_SEGMENTS[i].y0;
            double len = std::hypot(opt_segments[i].dx, opt_segments[i].dy);
            if (len > 1e-6) {
                opt_segments[i].nx = -opt_segments[i].dy / len;
                opt_segments[i].ny = opt_segments[i].dx / len;
            } else {
                opt_segments[i].nx = 0;
                opt_segments[i].ny = 0;
            }
        }

        for (auto& p : particles) {
            p.x = gaussian_sample(x, 2.0);
            p.y = gaussian_sample(y, 2.0);
            p.weight = 1.0 / NUM_PARTICLES;
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

            // Pre-calculate the current absolute robot heading for translations
            double robot_rad = degToRad(currOdom.theta);
            double robot_cos = std::cos(robot_rad);
            double robot_sin = std::sin(robot_rad);

            double cosP = std::cos(degToRad(prevOdom.theta));
            double sinP = std::sin(degToRad(prevOdom.theta));
            double dF = dX_global * sinP + dY_global * cosP;
            double dS = dX_global * cosP - dY_global * sinP;

            particle_mutex.take();

            for (auto& p : particles) {
                double dist = std::hypot(dF, dS);
                // Keep dTheta here to account for translation noise induced during heavy turning
                double transStd = PARAMS_TRANS_BASE + 0.08 * dist + 0.03 * std::abs(dTheta);
                
                double dx_l = dF + gaussian_sample(0, transStd);
                double dy_l = dS + gaussian_sample(0, transStd);

                p.x += dx_l * robot_sin + dy_l * robot_cos;
                p.y += dx_l * robot_cos - dy_l * robot_sin;
            }
            prevOdom = currOdom;

            std::vector<MCLDistanceSensor*> active;
            for (auto& s : Sensors) { s.Measure(); if (s.measurement > 0) active.push_back(&s); }

            if (!active.empty()) {
                double max_log = -1e9;
                std::vector<double> logs(NUM_PARTICLES);

                // PERFORMANCE BOOST: Pre-compute sensor offsets and ray vectors globally
                struct SensorPrecomp {
                    MCLDistanceSensor* s;
                    double dx, dy, offsetX, offsetY;
                };
                
                std::vector<SensorPrecomp> precomps;
                for (auto* s : active) {
                    SensorPrecomp sc;
                    sc.s = s;
                    sc.dx = robot_sin * s->cos_off + robot_cos * s->sin_off;
                    sc.dy = robot_cos * s->cos_off - robot_sin * s->sin_off;
                    sc.offsetX = s->LocalY * robot_sin + s->LocalX * robot_cos;
                    sc.offsetY = s->LocalY * robot_cos - s->LocalX * robot_sin;
                    precomps.push_back(sc);
                }

                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    double logL = 0.0;
                    
                    ValidMeas valid_measurements[4];
                    int valid_count = 0;

                    for (const auto& sc : precomps) {
                        double sensX = particles[i].x + sc.offsetX;
                        double sensY = particles[i].y + sc.offsetY;

                        RayHit hit = getRaycastDistance(sensX, sensY, sc.dx, sc.dy);
                        
                        if (hit.dist < 0) continue; 
                        double err = sc.s->measurement - hit.dist;

                        if (std::abs(err) > 18.0) continue;  

                        valid_measurements[valid_count++] = {hit.dist, sc.s->measurement, hit.weight};

                        double gaussian_prob = std::exp(-(err * err) / (2 * PARAMS_SENSOR_SIGMA * PARAMS_SENSOR_SIGMA));
                        
                        double p_abs = gaussian_prob * hit.weight;
                        logL += std::log(std::max(p_abs, 1e-6));
                    }

                    if (valid_count > 1) {
                        for (int a = 0; a < valid_count; ++a) {
                            for (int b = a + 1; b < valid_count; ++b) {
                                double predDiff = valid_measurements[a].pred - valid_measurements[b].pred;
                                double measDiff = valid_measurements[a].meas - valid_measurements[b].meas;
                                double diffErr = measDiff - predDiff;

                                double w = std::min(valid_measurements[a].weight, valid_measurements[b].weight);
                                double rel_prob = std::exp(-(diffErr * diffErr) / (2 * PARAMS_REL_SIGMA * PARAMS_REL_SIGMA));
                                
                                logL += std::log(std::max(rel_prob * w, 1e-6));
                            }
                        }
                    }

                    logs[i] = logL;
                    if (logL > max_log) max_log = logL;
                }

                double sumW = 0;
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    particles[i].weight = std::exp(logs[i] - max_log);
                    
                    if (particles[i].x < map_min_x || particles[i].x > map_max_x || 
                        particles[i].y < map_min_y || particles[i].y > map_max_y) {
                        particles[i].weight *= 0.01;
                    }
                    sumW += particles[i].weight;
                }
                for (auto& p : particles) p.weight /= (sumW > 1e-9 ? sumW : 1.0);
            }

            double mX = 0, mY = 0, neff_s = 0;
            for (const auto& p : particles) {
                mX += p.x * p.weight; 
                mY += p.y * p.weight;
                neff_s += p.weight * p.weight;
            }
            
            // IMU serves as the source of truth for heading
            global_X = mX; 
            global_Y = mY; 
            global_Theta = currOdom.theta; 
            
            double neff = (neff_s > 0) ? (1.0 / neff_s) : 0;
            global_Confidence = std::clamp(neff / NUM_PARTICLES, 0.0, 1.0);

            if (global_Confidence < RESAMPLE_THRESHOLD && (!active.empty() || global_Confidence < 0.15)) {
                double injectP = 0.02 + (1.0 - global_Confidence) * 0.15;
                if (global_Confidence < 0.2) injectP = 0.3; 

                int keep = NUM_PARTICLES * (1.0 - injectP);
                double r = std::uniform_real_distribution<double>(0, 1.0 / keep)(Random), c = particles[0].weight;
                int idx = 0;
                
                for (int i = 0; i < keep; ++i) {
                    double U = r + (double)i / keep;
                    while (U > c && idx < NUM_PARTICLES - 1) c += particles[++idx].weight;
                    
                    resample_buffer[i] = particles[idx];
                    resample_buffer[i].weight = 1.0 / NUM_PARTICLES;
                    
                    resample_buffer[i].x += gaussian_sample(0, 0.2);
                    resample_buffer[i].y += gaussian_sample(0, 0.2);
                }
                
                std::normal_distribution<double> fX(currOdom.x, 10.0);
                std::normal_distribution<double> fY(currOdom.y, 10.0);
                
                for (int i = keep; i < NUM_PARTICLES; ++i) {
                    double newX = std::clamp(fX(Random), map_min_x, map_max_x);
                    double newY = std::clamp(fY(Random), map_min_y, map_max_y);
                    resample_buffer[i] = {newX, newY, 1.0/NUM_PARTICLES};
                }
                particles = resample_buffer;
            }

            static uint32_t last_print = pros::millis();
            if (pros::millis() - last_print > 500) { 
                last_print = pros::millis();
                
                printf("Odom = (%.2f, %.2f, %.2f)\n", currOdom.x, currOdom.y, currOdom.theta);
                printf("MCL = (%.2f, %.2f)\n", global_X, global_Y);
                
                printf("P = [");
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    printf("(%.2f, %.2f)", particles[i].x, particles[i].y);
                    if (i != NUM_PARTICLES - 1) printf(", ");
                }
                printf("]\n\n");
            }
            particle_mutex.give(); 
            
            pros::delay(5); 
        }
    }
}