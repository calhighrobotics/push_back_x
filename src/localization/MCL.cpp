#include "MCL.h"
#include "globals.h" 
#include <algorithm>
#include <cmath>
#include <vector>
#include <optional>
#include <limits>

namespace MCL {
    double PARAMS_TRANS_BASE = 0.25;      
    double PARAMS_TRANS_GAIN = 0.025;      

    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0; 
    
    float w_slow = 0.0f, w_fast = 0;
    constexpr float ALPHA_SLOW = 0.001f; 
    constexpr float ALPHA_FAST = 0.1f;   

    constexpr float FIELD_SIZE = 140.42f;
    constexpr float HALF_SIZE = 0.5f * FIELD_SIZE;
    constexpr float FIELD_MIN = -HALF_SIZE;
    constexpr float FIELD_MAX = HALF_SIZE;
    
    constexpr float LOADER_X = 47.0f;
    constexpr float LOADER_RADIUS = 2.0f; 
    constexpr float LOADER_PADDING = 0.0f; 
    constexpr float LOADER_WIDTH = LOADER_RADIUS * 2.0f + LOADER_PADDING * 2.0f;
    constexpr float LOADER_LENGTH = LOADER_RADIUS * 2.0f + LOADER_PADDING;

    constexpr std::pair<float, float> LOADERS[4] = {
      {-LOADER_X, -(FIELD_SIZE / 2.0f) + LOADER_RADIUS + LOADER_PADDING / 2.0f},
      {-LOADER_X,  (FIELD_SIZE / 2.0f) - LOADER_RADIUS - LOADER_PADDING / 2.0f},
      { LOADER_X,  (FIELD_SIZE / 2.0f) - LOADER_RADIUS - LOADER_PADDING / 2.0f},
      { LOADER_X, -(FIELD_SIZE / 2.0f) + LOADER_RADIUS + LOADER_PADDING / 2.0f}
    };

    constexpr float GOAL_POST_X = 48.0f;
    constexpr float GOAL_POST_Y = 23.0f;
    constexpr float GOAL_POST_PADDING = 1.0f; 
    constexpr float GOAL_POST_WIDTH = 3.0f + GOAL_POST_PADDING * 2.0f;
    constexpr float GOAL_POST_LENGTH = 3.0f + GOAL_POST_PADDING * 2.0f;

    constexpr std::pair<float, float> GOAL_POSTS[4] = {
      {-GOAL_POST_X, -GOAL_POST_Y}, {-GOAL_POST_X,  GOAL_POST_Y},
      { GOAL_POST_X,  GOAL_POST_Y}, { GOAL_POST_X, -GOAL_POST_Y}
    };

    constexpr float MIDDLE_PADDING = 0.0f; 
    constexpr float MIDDLE_WIDTH = 2.0f + MIDDLE_PADDING * 2.0f;
    constexpr float MIDDLE_LENGTH = 2.0f + MIDDLE_PADDING * 2.0f;
    constexpr std::pair<float, float> MIDDLE_GOAL = {0.0f, 0.0f};

    pros::Mutex particle_mutex;

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

    struct Intersect {
        float dist;
        bool is_vertical;
    };

    inline std::optional<Intersect> check_bbox(float sx, float sy, float idx, float idy, float cx, float cy, float w, float h) {
        float hw = w * 0.5f, hh = h * 0.5f;
        float t1 = (cx - hw - sx) * idx;
        float t2 = (cx + hw - sx) * idx;
        float tmin_x = std::min(t1, t2);
        float tmax_x = std::max(t1, t2);
        
        float t3 = (cy - hh - sy) * idy;
        float t4 = (cy + hh - sy) * idy;
        float tmin_y = std::min(t3, t4);
        float tmax_y = std::max(t3, t4);
        
        float tmin = std::max(tmin_x, tmin_y);
        float tmax = std::min(tmax_x, tmax_y);
        
        if (tmax < 0.0f || tmin > tmax) return std::nullopt;
        float t;
        bool is_vert;
        if (tmin < 0.0f) {
            t = tmax;
            is_vert = (tmax_x < tmax_y);
        } else {
            t = tmin;
            is_vert = (tmin_x > tmin_y);
        }
        
        return Intersect{t, is_vert};
    }

    inline std::optional<Intersect> get_map_intersect(float sx, float sy, float idx, float idy) {
        float min_t = 100.0f;
        bool hit = false;
        bool is_vert = false;
        
        auto check = [&](float cx, float cy, float w, float h) {
            auto res = check_bbox(sx, sy, idx, idy, cx, cy, w, h);
            if (res && res->dist < min_t) {
                min_t = res->dist;
                is_vert = res->is_vertical;
                hit = true;
            }
        };
        
        check(0.0f, 0.0f, FIELD_SIZE, FIELD_SIZE);
        for (const auto& l : LOADERS) check(l.first, l.second, LOADER_WIDTH, LOADER_LENGTH);
        for (const auto& p : GOAL_POSTS) check(p.first, p.second, GOAL_POST_WIDTH, GOAL_POST_LENGTH);
        check(MIDDLE_GOAL.first, MIDDLE_GOAL.second, MIDDLE_WIDTH, MIDDLE_LENGTH);
        
        if (hit) return Intersect{min_t, is_vert};
        return std::nullopt;
    }

    struct Reading {
        float recorded;
        float inv_var;
        float std_dev;
        float rel_x, rel_y;
        float dir_x, dir_y;
        float idx, idy;

        Reading(float rec, float sdev, float rx, float ry, float dx, float dy, float ix, float iy)
            : recorded(rec), inv_var(-0.5f / (sdev * sdev)), std_dev(sdev), 
              rel_x(rx), rel_y(ry), dir_x(dx), dir_y(dy), idx(ix), idy(iy) {}
        
        inline float calculate_likelihood(const Intersect& inter) const {
            float error = recorded - inter.dist;
            float p_hit = std::exp(inv_var * error * error); 
            float cos_weight = inter.is_vertical ? std::abs(dir_x) : std::abs(dir_y);
            p_hit *= std::max(cos_weight, 0.05f); 
            
            constexpr float Z_HIT = 0.985f;
            constexpr float Z_RAND = 0.015f;
            constexpr float P_RAND = 1.0f / 140.0f; 
            
            return (Z_HIT * p_hit) + (Z_RAND * P_RAND);
        }
    };

    float particle_x[NUM_PARTICLES];
    float particle_y[NUM_PARTICLES];
    float particle_weights[NUM_PARTICLES];
    float temp_x[NUM_PARTICLES];
    float temp_y[NUM_PARTICLES];

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
            particle_x[i] = std::clamp((float)x + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_y[i] = std::clamp((float)y + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_weights[i] = 1.0f / NUM_PARTICLES;
        }
        w_slow = 1.0f / NUM_PARTICLES;
        w_fast = 1.0f / NUM_PARTICLES;
        particle_mutex.give();
    }

    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        uint32_t now = pros::millis(); 
        float Neff = NUM_PARTICLES;
        
        while (true) {
            uint32_t loop_start_micros = pros::micros(); 
            lemlib::Pose currOdom = chassis.getPose();

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
            
            double dist = std::hypot(dX_global, dY_global);
            double c = 1.0 - global_Confidence;
            
            double dynamic_base = PARAMS_TRANS_BASE + c * 0.3f;
            double dynamic_gain = PARAMS_TRANS_GAIN + c * 0.04f;
            double transStd = dynamic_base + dynamic_gain * dist + 0.15 * std::abs(dTheta);

            for (int i = 0; i < NUM_PARTICLES; i++) {
                float noise_F = rng.gaussian(transStd);
                float noise_S = rng.gaussian(transStd);

                float noise_X = noise_F * robot_cos - noise_S * robot_sin;
                float noise_Y = noise_F * robot_sin + noise_S * robot_cos;

                particle_x[i] += dX_global + noise_X;
                particle_y[i] += dY_global + noise_Y;

                particle_x[i] = std::clamp(particle_x[i], FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
                particle_y[i] = std::clamp(particle_y[i], FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            }

            std::vector<Reading> readings;
            for (auto& s : Sensors) {
                s.Measure();
                if (s.measurement > 0.0f && s.measurement < 100.0f) { 
                    
                    float rel_x = s.LocalX * robot_cos - s.LocalY * robot_sin;
                    float rel_y = s.LocalX * robot_sin + s.LocalY * robot_cos;

                    float dir_x = robot_cos * s.cos_off - robot_sin * s.sin_off;
                    float dir_y = robot_sin * s.cos_off + robot_cos * s.sin_off;
                    float mag = std::hypot(dir_x, dir_y);
                    dir_x /= mag;
                    dir_y /= mag;
                    float idx = (std::abs(dir_x) < 1e-6f) ? ((dir_x < 0) ? -1e6f : 1e6f) : (1.0f / dir_x);
                    float idy = (std::abs(dir_y) < 1e-6f) ? ((dir_y < 0) ? -1e6f : 1e6f) : (1.0f / dir_y);
                    float d = s.measurement;
                    float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
                    float dynamic_k = 1.0f + global_Confidence * 4.0f;          
                    float std_dev = std::max(bound / dynamic_k, 0.4f + 0.015f * d + 0.0004f * d * d);
                    auto odom_inter = get_map_intersect(currOdom.x + rel_x, currOdom.y + rel_y, idx, idy);
                    if (odom_inter) {
                        float error = std::abs(odom_inter->dist - d);
                        float gate = std::max(4.0f * std_dev, 10.0f + 20.0f * (1.0f - (float)global_Confidence));
                        if (error > gate) continue; 
                    } else {
                        continue; 
                    }

                    readings.emplace_back(d, std_dev, rel_x, rel_y, dir_x, dir_y, idx, idy);
                }
            }

            if (!readings.empty()) {
                float max_log_weight = -std::numeric_limits<float>::infinity();
                float log_weights[NUM_PARTICLES];
                
                for (int i = 0; i < NUM_PARTICLES; i++) {
                    float log_weight = 0.0f; 
                    float px = particle_x[i];
                    float py = particle_y[i];
                    
                    for (const auto& r : readings) {
                        auto predicted = get_map_intersect(px + r.rel_x, py + r.rel_y, r.idx, r.idy);
                        if (predicted) {
                            log_weight += std::log(r.calculate_likelihood(predicted.value()));
                        } else {
                            log_weight += std::log(0.001f); 
                        }
                    }
                    
                    log_weights[i] = log_weight;
                    if (log_weight > max_log_weight) max_log_weight = log_weight;
                }

                float weight_sum = 0.0f;
                for (int i = 0; i < NUM_PARTICLES; i++) {
                    particle_weights[i] = std::exp(log_weights[i] - max_log_weight);
                    weight_sum += particle_weights[i];
                }

                float avg_weight = 0.0f;
                if (weight_sum > 0.0f) {
                    avg_weight = weight_sum / NUM_PARTICLES;
                    float inv_weight_sum = 1.0f / weight_sum;
                    for (int i = 0; i < NUM_PARTICLES; i++) particle_weights[i] *= inv_weight_sum;
                } else {
                    float uniform_weight = 1.0f / NUM_PARTICLES;
                    for (int i = 0; i < NUM_PARTICLES; i++) particle_weights[i] = uniform_weight;
                    avg_weight = uniform_weight;
                }

                w_slow += ALPHA_SLOW * (avg_weight - w_slow);
                w_fast += ALPHA_FAST * (avg_weight - w_fast);
            }
            float mX = 0, mY = 0;
            float neff_sum = 0.0f;

            for (int i = 0; i < NUM_PARTICLES; i++) {
                mX += particle_x[i] * particle_weights[i];
                mY += particle_y[i] * particle_weights[i];
                neff_sum += particle_weights[i] * particle_weights[i];
            }
            
            global_X = mX; 
            global_Y = mY; 
            global_Theta = currOdom.theta; 
            
            Neff = (neff_sum > 0.0f) ? (1.0f / neff_sum) : 0.0f;
            global_Confidence = std::clamp(Neff / NUM_PARTICLES, 0.0f, 1.0f); 

            if (Neff < 0.5f * NUM_PARTICLES) {
                float injectP = std::max(0.0f, 1.0f - (w_fast / std::max(w_slow, 1e-6f)));
                injectP = std::clamp(injectP, 0.0f, 0.30f); 

                int keep_count = NUM_PARTICLES * (1.0f - injectP);
                if (keep_count > 0) {
                    float r = rng.next_f32() * (1.0f / keep_count);
                    float c = particle_weights[0];
                    int i = 0;
                    
                    constexpr float JITTER = 0.15f;
                    
                    for (int m = 0; m < keep_count; m++) {
                        float U = r + m * (1.0f / keep_count);
                        while (U > c && i < NUM_PARTICLES - 1) {
                            i++;
                            c += particle_weights[i];
                        }
                        temp_x[m] = particle_x[i] + rng.gaussian(JITTER); 
                        temp_y[m] = particle_y[i] + rng.gaussian(JITTER);
                    }
                }

                float spread = 10.0f + (1.0f - global_Confidence) * 15.0f;
                for (int m = keep_count; m < NUM_PARTICLES; m++) {
                    temp_x[m] = std::clamp((float)currOdom.x + rng.gaussian(spread), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
                    temp_y[m] = std::clamp((float)currOdom.y + rng.gaussian(spread), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
                }
                std::copy(temp_x, temp_x + NUM_PARTICLES, particle_x);
                std::copy(temp_y, temp_y + NUM_PARTICLES, particle_y);
                float uniform = 1.0f / NUM_PARTICLES;
                for (int m=0; m<NUM_PARTICLES; m++) particle_weights[m] = uniform;
            }
            constexpr float ALPHA_MAX = 0.12f; 
            float alpha = ALPHA_MAX * (global_Confidence * global_Confidence);

            float error_x = global_X - currOdom.x;
            float error_y = global_Y - currOdom.y;

            lemlib::Pose latestOdom = chassis.getPose();

            float blended_x = latestOdom.x + (alpha * error_x);
            float blended_y = latestOdom.y + (alpha * error_y);

            float correction_dist = std::hypot(alpha * error_x, alpha * error_y);

            if (correction_dist > 0.05f) {
                chassis.setPose(blended_x, blended_y, latestOdom.theta);
                prevOdom.x = blended_x;
                prevOdom.y = blended_y;
                prevOdom.theta = latestOdom.theta; 
            } else {
                prevOdom = latestOdom; 
            }

            particle_mutex.give(); 
            pros::Task::delay_until(&now, 30); 
        }
    }
}