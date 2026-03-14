#include "MCL.h"
#include "globals.h" 
#include <algorithm>
#include <cmath>
#include <vector>
#include <optional>

namespace MCL {
    // Filter tuning params for Motor Encoders
    double PARAMS_TRANS_BASE = 0.5;      
    double PARAMS_TRANS_GAIN = 0.09;      

    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0; 
    
    // Official VEX Competition Field Constants
    constexpr float FIELD_SIZE = 140.42f;
    constexpr float HALF_SIZE = 0.5f * FIELD_SIZE;
    constexpr float FIELD_MIN = -HALF_SIZE;
    constexpr float FIELD_MAX = HALF_SIZE;

    // --- Obstacle definitions for pre-filtering (Low-Sensor Setup) ---
    constexpr float LOADER_X = 47.0f;
    constexpr float LOADER_RADIUS = 3.0f;
    constexpr float LOADER_PADDING = 0.5f;
    constexpr float LOADER_WIDTH = LOADER_RADIUS * 2.0f + LOADER_PADDING * 2.0f;
    constexpr float LOADER_LENGTH = LOADER_RADIUS * 2.0f + LOADER_PADDING;

    constexpr std::pair<float, float> LOADERS[4] = {
      {-LOADER_X, -(FIELD_SIZE / 2.0f) + LOADER_RADIUS + LOADER_PADDING / 2.0f},
      {-LOADER_X,  (FIELD_SIZE / 2.0f) - LOADER_RADIUS - LOADER_PADDING / 2.0f},
      { LOADER_X,  (FIELD_SIZE / 2.0f) - LOADER_RADIUS - LOADER_PADDING / 2.0f},
      { LOADER_X, -(FIELD_SIZE / 2.0f) + LOADER_RADIUS + LOADER_PADDING / 2.0f}
    };

    // The posts holding up the long goals (Updated to 3x3 for low sensors)
    constexpr float GOAL_POST_X = 48.0f;
    constexpr float GOAL_POST_Y = 23.0f;
    constexpr float GOAL_POST_PADDING = 1.0f; // 1-inch safety buffer
    constexpr float GOAL_POST_WIDTH = 3.0f + GOAL_POST_PADDING * 2.0f;
    constexpr float GOAL_POST_LENGTH = 3.0f + GOAL_POST_PADDING * 2.0f;

    constexpr std::pair<float, float> GOAL_POSTS[4] = {
      {-GOAL_POST_X, -GOAL_POST_Y}, {-GOAL_POST_X,  GOAL_POST_Y},
      { GOAL_POST_X,  GOAL_POST_Y}, { GOAL_POST_X, -GOAL_POST_Y}
    };

    // The middle goal / center elevation structure (Updated to 5x5 for low sensors)
    constexpr float MIDDLE_PADDING = 1.0f; 
    constexpr float MIDDLE_WIDTH = 5.0f + MIDDLE_PADDING * 2.0f;
    constexpr float MIDDLE_LENGTH = 5.0f + MIDDLE_PADDING * 2.0f;
    constexpr std::pair<float, float> MIDDLE_GOAL = {0.0f, 0.0f};

    pros::Mutex particle_mutex;

    // --- Blazingly Fast RNG ---
    struct XorShift32 {
        uint32_t state;
        inline XorShift32(uint32_t seed = pros::micros()) : state(seed == 0 ? 0x12345678 : seed) {}
        inline uint32_t next_u32() {
            uint32_t x = state;
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            state = x; return x;
        }
        inline float next_f32() { return (next_u32() >> 8) * (1.0f / (1u << 24)); }
        inline float range_f32(float min, float max) { return min + (max - min) * next_f32(); }
        inline float gaussian(float std_dev) {
            float u1 = std::max(next_f32(), 1e-12f);
            float u2 = next_f32();
            float r = std::sqrt(-2.0f * std::log(u1));
            float theta = 2.0f * M_PI * u2;
            return r * std::cos(theta) * std_dev;
        }
    };
    XorShift32 rng;

    struct Point { float x, y; };

    struct Line {
        Point start;
        Point end;
        
        // ==========================================
        // Robust Slab Method Raycaster
        // Handles both inside-out (walls) and outside-in (obstacles) perfectly.
        // ==========================================
        inline std::optional<float> square_intersect_distance(float center_x, float center_y, float width, float height) const {
            float half_w = width * 0.5f;
            float half_h = height * 0.5f;
            float min_x = center_x - half_w;
            float max_x = center_x + half_w;
            float min_y = center_y - half_h;
            float max_y = center_y + half_h;

            float dx = end.x - start.x;
            float dy = end.y - start.y;

            float tmin = -std::numeric_limits<float>::infinity();
            float tmax = std::numeric_limits<float>::infinity();

            if (std::abs(dx) < 1e-6f) {
                if (start.x < min_x || start.x > max_x) return std::nullopt;
            } else {
                float inv_dx = 1.0f / dx;
                float t1 = (min_x - start.x) * inv_dx;
                float t2 = (max_x - start.x) * inv_dx;
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
            }

            if (std::abs(dy) < 1e-6f) {
                if (start.y < min_y || start.y > max_y) return std::nullopt;
            } else {
                float inv_dy = 1.0f / dy;
                float t1 = (min_y - start.y) * inv_dy;
                float t2 = (max_y - start.y) * inv_dy;
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
            }

            if (tmin > tmax || tmax < 0.0f) return std::nullopt;

            float t = (tmin < 0.0f) ? tmax : tmin;
            return t * std::hypot(dx, dy);
        }
    };

    struct Reading {
        float recorded;
        float inv_var;
        Point relative_pos;
        Point proj_relative;

        Reading(float recorded, float std_dev, Point relative_pos, Point proj_relative)
            : recorded(recorded), inv_var(-0.5f / (std_dev * std_dev)), relative_pos(relative_pos), proj_relative(proj_relative) {}
        
        inline std::optional<float> predict(Point particle_pos) const {
            return Line{
                Point{relative_pos.x + particle_pos.x, relative_pos.y + particle_pos.y}, 
                Point{proj_relative.x + particle_pos.x, proj_relative.y + particle_pos.y}
            }.square_intersect_distance(0.0f, 0.0f, FIELD_SIZE, FIELD_SIZE);
        }
    };

    // --- Structure of Arrays (SoA) Particle State ---
    float particle_x[NUM_PARTICLES];
    float particle_y[NUM_PARTICLES];
    float particle_weights[NUM_PARTICLES];
    float temp_x[NUM_PARTICLES];
    float temp_y[NUM_PARTICLES];
    float temp_weights[NUM_PARTICLES];

    // --- Sensors ---
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
    
    // Updated with your new sensor offsets
    std::vector<MCLDistanceSensor> Sensors = {
        MCLDistanceSensor(frontDistance, -2.5, 3.3, 0),  
        MCLDistanceSensor(leftDistance, -6.2, -4.6, 90), 
        MCLDistanceSensor(rightDistance, 6.2, 4.6, -90), 
        MCLDistanceSensor(backDistance, -1.5, -0.6, 180), 
    };

    inline double degToRad(double deg) { return deg * M_PI / 180.0; }
    inline double wrapAngleDeg(double angle) {
        angle = std::fmod(angle + 180.0, 360.0);
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    void StartMCL(double x, double y) {
        particle_mutex.take();
        rng = XorShift32(pros::micros());
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i] = std::clamp((float)x + rng.gaussian(4.0f), FIELD_MIN, FIELD_MAX);
            particle_y[i] = std::clamp((float)y + rng.gaussian(4.0f), FIELD_MIN, FIELD_MAX);
            particle_weights[i] = 1.0f / NUM_PARTICLES;
        }
        particle_mutex.give();
    }

    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        uint32_t now = pros::millis(); 
        double last_exec_ms = 0.0; 
        
        while (true) {
            uint32_t loop_start_micros = pros::micros(); 
            
            // 1. Grab odometry at the START of the calculations
            lemlib::Pose currOdom = chassis.getPose();

            // Use GLOBAL deltas to preserve Lemlib's curved arc tracking perfectly
            double dX_global = currOdom.x - prevOdom.x;
            double dY_global = currOdom.y - prevOdom.y;
            double dTheta = wrapAngleDeg(currOdom.theta - prevOdom.theta);

            double robot_rad = degToRad(currOdom.theta);
            double robot_cos = std::cos(robot_rad);
            double robot_sin = std::sin(robot_rad);

            particle_mutex.take();

            if (std::abs(dX_global) < 0.05 && std::abs(dY_global) < 0.05 && std::abs(dTheta) < 0.5) {
                global_Theta = currOdom.theta; 
                prevOdom = currOdom;
                
                particle_mutex.give(); 
                pros::Task::delay_until(&now, 30);
                continue; 
            }
            
            // ================== 1. ADAPTIVE PREDICT ==================
            double dist = std::hypot(dX_global, dY_global);
            double c = 1.0 - global_Confidence;
            
            double dynamic_base = PARAMS_TRANS_BASE + c * c * 0.9;
            double dynamic_gain = PARAMS_TRANS_GAIN + c * c * 0.06;
            
            // Increased dTheta multiplier to 0.15 for Motor Encoders!
            double transStd = dynamic_base + dynamic_gain * dist + 0.15 * std::abs(dTheta);

            for (int i = 0; i < NUM_PARTICLES; i++) {
                // Generate noise locally
                float noise_F = rng.gaussian(transStd);
                float noise_S = rng.gaussian(transStd);

                // Rotate noise to global frame
                float noise_X = noise_F * robot_sin + noise_S * robot_cos;
                float noise_Y = noise_F * robot_cos - noise_S * robot_sin;

                // Add Lemlib's precise global delta + the generated noise
                particle_x[i] += dX_global + noise_X;
                particle_y[i] += dY_global + noise_Y;

                particle_x[i] = std::clamp(particle_x[i], FIELD_MIN, FIELD_MAX);
                particle_y[i] = std::clamp(particle_y[i], FIELD_MIN, FIELD_MAX);
            }

            // ================== 2. SENSOR PRE-FILTERING ==================
            std::vector<Reading> readings;
            for (auto& s : Sensors) {
                s.Measure();
                if (s.measurement > 0) {
                    float offsetX = s.LocalY * robot_sin + s.LocalX * robot_cos;
                    float offsetY = s.LocalY * robot_cos - s.LocalX * robot_sin;
                    Point rel_pos = {offsetX, offsetY};

                    float dirX = robot_sin * s.cos_off + robot_cos * s.sin_off;
                    float dirY = robot_cos * s.cos_off - robot_sin * s.sin_off;
                    Point ray_pt = {offsetX + dirX, offsetY + dirY};

                    Line ray{Point{(float)currOdom.x + offsetX, (float)currOdom.y + offsetY}, 
                             Point{(float)currOdom.x + offsetX + dirX, (float)currOdom.y + offsetY + dirY}};

                    bool hits_obstacle = false;
                    
                    for(auto& loader : LOADERS) {
                        if (ray.square_intersect_distance(loader.first, loader.second, LOADER_WIDTH, LOADER_LENGTH)) hits_obstacle = true;
                    }
                    for(auto& post : GOAL_POSTS) {
                        if (ray.square_intersect_distance(post.first, post.second, GOAL_POST_WIDTH, GOAL_POST_LENGTH)) hits_obstacle = true;
                    }
                    if (ray.square_intersect_distance(MIDDLE_GOAL.first, MIDDLE_GOAL.second, MIDDLE_WIDTH, MIDDLE_LENGTH)) hits_obstacle = true;
                    
                    if (!hits_obstacle) {
                        auto expected_wall_dist = ray.square_intersect_distance(0.0f, 0.0f, FIELD_SIZE, FIELD_SIZE);
                        
                        float rejection_threshold = 4.0f + (1.0f - global_Confidence) * 12.0f; 
                        
                        if (expected_wall_dist.has_value() && std::abs(expected_wall_dist.value() - s.measurement) > rejection_threshold) {
                            hits_obstacle = true; 
                        }
                    }

                    if (!hits_obstacle) {
                        float d = s.measurement;
                        float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
                        float dynamic_k = 1.0f + global_Confidence * 4.0f;
                        float std_dev = std::max(bound / dynamic_k, 0.5f);

                        readings.emplace_back(d, std_dev, rel_pos, ray_pt);
                    }
                }
            }

            // ================== 3. UPDATE ==================
            if (!readings.empty()) {
                float max_weight = 0.0f;
                
                for (int i = 0; i < NUM_PARTICLES; i++) {
                    float weight = particle_weights[i]; 
                    
                    for (const auto& reading : readings) {
                        auto predicted = reading.predict(Point{particle_x[i], particle_y[i]});
                        if (predicted.has_value()) {
                            float error = reading.recorded - predicted.value();
                            float likelihood = std::exp(reading.inv_var * error * error) + 0.05f;
                            weight *= likelihood;
                            
                            if (weight <= 0.0f) break;
                        } else {
                            weight = 0.0f;
                            break;
                        }
                    }

                    if (!std::isfinite(weight) || weight < 0.0f) weight = 0.0f;
                    particle_weights[i] = weight;
                    if (weight > max_weight) max_weight = weight;
                }

                float weight_sum = 0.0f;
                if (max_weight > 0.0f) {
                    for (int i = 0; i < NUM_PARTICLES; i++) {
                        particle_weights[i] /= max_weight;
                        weight_sum += particle_weights[i];
                    }
                }

                if (weight_sum <= 0.0f) {
                    float uniform_weight = 1.0f / NUM_PARTICLES;
                    for (int i = 0; i < NUM_PARTICLES; i++) particle_weights[i] = uniform_weight;
                } else {
                    float inv_weight_sum = 1.0f / weight_sum;
                    for (int i = 0; i < NUM_PARTICLES; i++) particle_weights[i] *= inv_weight_sum;
                }
            }

            // ================== 4. ESTIMATE ==================
            float mX = 0, mY = 0, neff_s = 0;
            for (int i = 0; i < NUM_PARTICLES; i++) {
                mX += particle_x[i] * particle_weights[i];
                mY += particle_y[i] * particle_weights[i];
                neff_s += particle_weights[i] * particle_weights[i];
            }
            
            global_X = mX; 
            global_Y = mY; 
            global_Theta = currOdom.theta; 
            
            float neff = (neff_s > 0) ? (1.0f / neff_s) : 0;
            global_Confidence = std::clamp((double)(neff / NUM_PARTICLES), 0.0, 1.0);

            // ================== 5. RESAMPLE & RECOVERY ==================
            if (global_Confidence < RESAMPLE_THRESHOLD && (!readings.empty() || global_Confidence < 0.15)) {
                float injectP = 0.02f + (1.0f - global_Confidence) * 0.15f;
                if (global_Confidence < 0.2f) injectP = 0.3f; 

                int keep = NUM_PARTICLES * (1.0f - injectP);
                float inv_keep = 1.0f / keep;
                float offset = rng.next_f32() * inv_keep;

                float cumulative_weight = particle_weights[0];
                size_t idx = 0;

                for (int i = 0; i < keep; i++) {
                    float sample = offset + i * inv_keep;
                    while (sample > cumulative_weight && idx < NUM_PARTICLES - 1) {
                        idx++;
                        cumulative_weight += particle_weights[idx];
                    }
                    temp_x[i] = particle_x[idx] + rng.gaussian(0.2f);
                    temp_y[i] = particle_y[idx] + rng.gaussian(0.2f);
                    temp_weights[i] = 1.0f / NUM_PARTICLES;
                }

                float spread = 10.0f + (1.0f - global_Confidence) * 15.0f;
                for (int i = keep; i < NUM_PARTICLES; i++) {
                    temp_x[i] = std::clamp((float)currOdom.x + rng.gaussian(spread), FIELD_MIN, FIELD_MAX);
                    temp_y[i] = std::clamp((float)currOdom.y + rng.gaussian(spread), FIELD_MIN, FIELD_MAX);
                    temp_weights[i] = 1.0f / NUM_PARTICLES;
                }

                std::copy(temp_x, temp_x + NUM_PARTICLES, particle_x);
                std::copy(temp_y, temp_y + NUM_PARTICLES, particle_y);
                std::copy(temp_weights, temp_weights + NUM_PARTICLES, particle_weights);
            }

            // ================== 6. APPLY CORRECTION SAFELY ==================
            constexpr float ALPHA_MAX = 0.05f; 
            float alpha = ALPHA_MAX * (global_Confidence * global_Confidence);

            // Calculate error based on the odometry from the START of the math
            float error_x = global_X - currOdom.x;
            float error_y = global_Y - currOdom.y;

            // Fetch the FRESHEST odometry (since the robot moved during the math)
            lemlib::Pose latestOdom = chassis.getPose();

            // Apply the blend to the LATEST odometry so we don't erase travel distance
            float blended_x = latestOdom.x + (alpha * error_x);
            float blended_y = latestOdom.y + (alpha * error_y);

            // Calculate how far we are actually shifting the robot right now
            float correction_dist = std::hypot(alpha * error_x, alpha * error_y);

            // DEADBAND: Only update Lemlib if the shift is larger than 0.05 inches.
            if (correction_dist > 0.05f) {
                chassis.setPose(blended_x, blended_y, latestOdom.theta);
                prevOdom.x = blended_x;
                prevOdom.y = blended_y;
                prevOdom.theta = latestOdom.theta; 
            } else {
                // If we didn't update, just track the latest odom for the next loop's delta
                prevOdom = latestOdom; 
            }
            
            // Calculate exact execution time
            last_exec_ms = (pros::micros() - loop_start_micros) / 1000.0;

            // ================== LOGGING & TIMING ==================
            static uint32_t last_log = pros::millis();
            if (pros::millis() - last_log > 50) { 
                last_log = pros::millis();
                printf("Odom: %.2f, %.2f | MCL: %.2f, %.2f | Exec: %.2f ms\n", 
                        latestOdom.x, latestOdom.y, global_X, global_Y, last_exec_ms);
                fflush(stdout); 
            }

            particle_mutex.give(); 
            pros::Task::delay_until(&now, 30); 
        }
    }
}