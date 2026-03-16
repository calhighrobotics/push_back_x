#include "MCL.h"
#include "globals.h" 
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
#include <queue>
#include "Eigen/Dense"

namespace MCL {
    double PARAMS_TRANS_BASE = 0.25;      
    double PARAMS_TRANS_GAIN = 0.025;      

    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0; 
    
    float w_slow = 0.0f;
    float w_fast = 0.0f;
    constexpr float ALPHA_SLOW = 0.001f; 
    constexpr float ALPHA_FAST = 0.1f;   

    constexpr float FIELD_SIZE = 140.42f;
    constexpr float HALF_SIZE = 0.5f * FIELD_SIZE;
    constexpr float FIELD_MIN = -HALF_SIZE;
    constexpr float FIELD_MAX = HALF_SIZE;

    constexpr int GRID_RES = 1; 
    constexpr int GRID_SIZE = 141; 
    float distance_map[GRID_SIZE][GRID_SIZE];
    bool map_initialized = false;

    constexpr float LOADER_X = 47.0f;
    constexpr float LOADER_RADIUS = 3.0f;
    constexpr float LOADER_PADDING = 0.5f;
    constexpr float LOADER_WIDTH = LOADER_RADIUS * 2.0f + LOADER_PADDING * 2.0f;
    constexpr float LOADER_LENGTH = LOADER_RADIUS * 2.0f + LOADER_PADDING;

    constexpr float GOAL_POST_X = 48.0f;
    constexpr float GOAL_POST_Y = 23.0f;
    constexpr float GOAL_POST_PADDING = 1.0f; 
    constexpr float GOAL_POST_WIDTH = 3.0f + GOAL_POST_PADDING * 2.0f;
    constexpr float GOAL_POST_LENGTH = 3.0f + GOAL_POST_PADDING * 2.0f;

    constexpr float MIDDLE_PADDING = 1.0f; 
    constexpr float MIDDLE_WIDTH = 5.0f + MIDDLE_PADDING * 2.0f;
    constexpr float MIDDLE_LENGTH = 5.0f + MIDDLE_PADDING * 2.0f;
    constexpr std::pair<float, float> MIDDLE_GOAL = {0.0f, 0.0f};

    struct Obstacle { float cx, cy, hw, hh; };
    constexpr Obstacle FIELD_OBSTACLES[9] = {
        {-LOADER_X, -(FIELD_SIZE / 2.0f) + LOADER_RADIUS + LOADER_PADDING / 2.0f, LOADER_WIDTH * 0.5f, LOADER_LENGTH * 0.5f},
        {-LOADER_X,  (FIELD_SIZE / 2.0f) - LOADER_RADIUS - LOADER_PADDING / 2.0f, LOADER_WIDTH * 0.5f, LOADER_LENGTH * 0.5f},
        { LOADER_X,  (FIELD_SIZE / 2.0f) - LOADER_RADIUS - LOADER_PADDING / 2.0f, LOADER_WIDTH * 0.5f, LOADER_LENGTH * 0.5f},
        { LOADER_X, -(FIELD_SIZE / 2.0f) + LOADER_RADIUS + LOADER_PADDING / 2.0f, LOADER_WIDTH * 0.5f, LOADER_LENGTH * 0.5f},
        {-GOAL_POST_X, -GOAL_POST_Y, GOAL_POST_WIDTH * 0.5f, GOAL_POST_LENGTH * 0.5f},
        {-GOAL_POST_X,  GOAL_POST_Y, GOAL_POST_WIDTH * 0.5f, GOAL_POST_LENGTH * 0.5f},
        { GOAL_POST_X,  GOAL_POST_Y, GOAL_POST_WIDTH * 0.5f, GOAL_POST_LENGTH * 0.5f},
        { GOAL_POST_X, -GOAL_POST_Y, GOAL_POST_WIDTH * 0.5f, GOAL_POST_LENGTH * 0.5f},
        {MIDDLE_GOAL.first, MIDDLE_GOAL.second, MIDDLE_WIDTH * 0.5f, MIDDLE_LENGTH * 0.5f}
    };

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

    void init_distance_map() {
        if (map_initialized) return;
        
        std::queue<std::pair<int, int>> q;

        for (int x = 0; x < GRID_SIZE; x++) {
            for (int y = 0; y < GRID_SIZE; y++) {
                distance_map[x][y] = 999.0f;
                
                float world_x = (x * GRID_RES) - HALF_SIZE;
                float world_y = (y * GRID_RES) - HALF_SIZE;

                bool is_obstacle = false;
                
                if (world_x <= FIELD_MIN + 1.0f || world_x >= FIELD_MAX - 1.0f ||
                    world_y <= FIELD_MIN + 1.0f || world_y >= FIELD_MAX - 1.0f) {
                    is_obstacle = true;
                } else {
                    for (int i = 0; i < 9; i++) {
                        const auto& obs = FIELD_OBSTACLES[i];
                        if (std::abs(world_x - obs.cx) <= obs.hw && 
                            std::abs(world_y - obs.cy) <= obs.hh) {
                            is_obstacle = true;
                            break;
                        }
                    }
                }

                if (is_obstacle) {
                    distance_map[x][y] = 0.0f;
                    q.push({x, y});
                }
            }
        }

        int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
        float cost[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f}; 

        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();

            float current_dist = distance_map[cx][cy];

            for (int i = 0; i < 8; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
                    float new_dist = current_dist + cost[i] * GRID_RES;
                    if (new_dist < distance_map[nx][ny]) {
                        distance_map[nx][ny] = new_dist;
                        q.push({nx, ny});
                    }
                }
            }
        }
        map_initialized = true;
    }

    Eigen::ArrayXf particle_x(NUM_PARTICLES);
    Eigen::ArrayXf particle_y(NUM_PARTICLES);
    Eigen::ArrayXf particle_weights(NUM_PARTICLES);
    Eigen::ArrayXf temp_x(NUM_PARTICLES);
    Eigen::ArrayXf temp_y(NUM_PARTICLES);
    Eigen::ArrayXf log_weights(NUM_PARTICLES); 
    Eigen::ArrayXf noise_X(NUM_PARTICLES);     
    Eigen::ArrayXf noise_Y(NUM_PARTICLES);

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
        init_distance_map(); 

        particle_mutex.take();
        rng = XorShift32(pros::micros());
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i] = std::clamp((float)x + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_y[i] = std::clamp((float)y + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
        }
        particle_weights.fill(1.0f / NUM_PARTICLES); 
        w_slow = 1.0f / NUM_PARTICLES;
        w_fast = 1.0f / NUM_PARTICLES;
        particle_mutex.give();
    }

    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        uint32_t now = pros::millis(); 
        float Neff = NUM_PARTICLES;
        
        while (true) {
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
            
            constexpr int MIN_PARTICLES = 150;
            int current_particles = MIN_PARTICLES + (int)((1.0f - global_Confidence) * (NUM_PARTICLES - MIN_PARTICLES));
            current_particles = std::clamp(current_particles, MIN_PARTICLES, NUM_PARTICLES);

            double dist = std::hypot(dX_global, dY_global);
            double c = 1.0 - global_Confidence;
            
            double dynamic_base = PARAMS_TRANS_BASE + c * 0.3f;
            double dynamic_gain = PARAMS_TRANS_GAIN + c * 0.04f;
            double transStd = dynamic_base + dynamic_gain * dist + 0.15 * std::abs(dTheta);

            for (int i = 0; i < current_particles; i++) {
                float noise_F = rng.gaussian(transStd);
                float noise_S = rng.gaussian(transStd);
                noise_X[i] = noise_F * robot_cos - noise_S * robot_sin;
                noise_Y[i] = noise_F * robot_sin + noise_S * robot_cos;
            }

            particle_x.head(current_particles) = (particle_x.head(current_particles) + dX_global + noise_X.head(current_particles)).cwiseMax(FIELD_MIN + 0.1f).cwiseMin(FIELD_MAX - 0.1f);
            particle_y.head(current_particles) = (particle_y.head(current_particles) + dY_global + noise_Y.head(current_particles)).cwiseMax(FIELD_MIN + 0.1f).cwiseMin(FIELD_MAX - 0.1f);

            bool took_measurements = false;

            for (auto& s : Sensors) {
                s.Measure();
                if (s.measurement > 0.0f && s.measurement < FIELD_SIZE) { 
                    took_measurements = true;
                    
                    float rel_x = robot_cos * s.LocalX - robot_sin * s.LocalY;
                    float rel_y = robot_sin * s.LocalX + robot_cos * s.LocalY;
                    float dir_x = robot_cos * s.cos_off - robot_sin * s.sin_off;
                    float dir_y = robot_sin * s.cos_off + robot_cos * s.sin_off;

                    float d = s.measurement;
                    float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
                    float dynamic_k = 1.0f + global_Confidence * 4.0f;
                    float std_dev = std::max(bound / dynamic_k, 0.4f + 0.015f * d + 0.0004f * d * d);
                    float variance2 = 2.0f * (std_dev * std_dev);

                    constexpr float Z_HIT = 0.95f; 
                    constexpr float Z_RAND = 0.05f;
                    constexpr float P_RAND = 1.0f / 140.0f; 

                    for (int i = 0; i < current_particles; i++) {
                        float end_x = particle_x[i] + rel_x + (dir_x * d);
                        float end_y = particle_y[i] + rel_y + (dir_y * d);

                        int gx = std::clamp((int)((end_x + HALF_SIZE) / GRID_RES), 0, GRID_SIZE - 1);
                        int gy = std::clamp((int)((end_y + HALF_SIZE) / GRID_RES), 0, GRID_SIZE - 1);

                        float dist_to_wall = distance_map[gx][gy];
                        float p_hit = std::exp(-(dist_to_wall * dist_to_wall) / variance2);
                        
                        float prob = (Z_HIT * p_hit) + (Z_RAND * P_RAND);
                        
                        if (i == 0) log_weights[i] = std::log(prob); 
                        else log_weights[i] += std::log(prob);
                    }
                }
            }

            if (took_measurements) {
                float max_log_weight = log_weights.head(current_particles).maxCoeff();
                particle_weights.head(current_particles) = (log_weights.head(current_particles) - max_log_weight).cwiseMax(-80.0f).exp();
                
                float weight_sum = particle_weights.head(current_particles).sum();
                float avg_weight = 0.0f;

                if (weight_sum > 0.0f) {
                    avg_weight = weight_sum / current_particles;
                    particle_weights.head(current_particles) /= weight_sum; 
                } else {
                    float uniform_weight = 1.0f / current_particles;
                    particle_weights.head(current_particles).fill(uniform_weight); 
                    avg_weight = uniform_weight;
                }

                w_slow += ALPHA_SLOW * (avg_weight - w_slow);
                w_fast += ALPHA_FAST * (avg_weight - w_fast);
            } else {
                particle_weights.head(current_particles).fill(1.0f / current_particles);
            }

            float mX = (particle_x.head(current_particles) * particle_weights.head(current_particles)).sum();
            float mY = (particle_y.head(current_particles) * particle_weights.head(current_particles)).sum();
            float neff_sum = (particle_weights.head(current_particles) * particle_weights.head(current_particles)).sum();
            
            global_X = mX; 
            global_Y = mY; 
            global_Theta = currOdom.theta; 
            
            Neff = (neff_sum > 0.0f) ? (1.0f / neff_sum) : 0.0f;
            global_Confidence = std::clamp(Neff / current_particles, 0.0f, 1.0f); 

            if (Neff < 0.5f * current_particles) {
                float injectP = std::max(0.0f, 1.0f - (w_fast / std::max(w_slow, 1e-6f)));
                injectP = std::clamp(injectP, 0.0f, 0.30f); 

                int keep_count = current_particles * (1.0f - injectP);
                if (keep_count > 0) {
                    float r = rng.next_f32() * (1.0f / keep_count);
                    float c = particle_weights[0];
                    int i = 0;
                    
                    constexpr float JITTER = 0.15f;
                    
                    for (int m = 0; m < keep_count; m++) {
                        float U = r + m * (1.0f / keep_count);
                        while (U > c && i < current_particles - 1) {
                            i++;
                            c += particle_weights[i];
                        }
                        temp_x[m] = particle_x[i] + rng.gaussian(JITTER); 
                        temp_y[m] = particle_y[i] + rng.gaussian(JITTER);
                    }
                }

                for (int m = keep_count; m < current_particles; m++) {
                    temp_x[m] = rng.range_f32(FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
                    temp_y[m] = rng.range_f32(FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
                }

                particle_x.head(current_particles) = temp_x.head(current_particles);
                particle_y.head(current_particles) = temp_y.head(current_particles);
                particle_weights.head(current_particles).fill(1.0f / current_particles);
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