#include "MCL.h"
#include "globals.h" 
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
#include <cstdio>
#include <mutex>

namespace MCL {

    double PARAMS_TRANS_BASE = 0.15;   
    double PARAMS_TRANS_GAIN = 0.08;  

    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0;

    float w_slow = 0.0f, w_fast = 0.0f;
    constexpr float ALPHA_SLOW = 0.001f;
    constexpr float ALPHA_FAST = 0.1f;

    constexpr float Z_HIT = 0.85f;     
    constexpr float Z_SHORT = 0.10f;   
    constexpr float Z_RAND = 0.05f;    
    constexpr float LAMBDA_SHORT = 0.1f; 

    constexpr float FIELD_SIZE = 140.42f; 
    constexpr float HALF_SIZE  = FIELD_SIZE * 0.5f;  
    constexpr float FIELD_MIN  = -HALF_SIZE;
    constexpr float FIELD_MAX  =  HALF_SIZE;

    constexpr float MAX_SENSOR_READING = 65.0f;  

    float particle_x[NUM_PARTICLES];
    float particle_y[NUM_PARTICLES];
    float particle_theta[NUM_PARTICLES]; 
    float particle_weights[NUM_PARTICLES];

    pros::Mutex particle_mutex;

    struct SensorConfig {
        float x;      
        float y;      
        float angle;  
    };

    const std::vector<SensorConfig> SENSOR_CONFIGS = {
        { -1.75f,  2.8f,   0.0f },    // 0: front
        {  4.6f,   4.3f,  90.0f },    // 1: right
        {  1.75f, -0.6f, 180.0f },    // 2: back
        { -5.1f,   4.3f, -90.0f }     // 3: left
    };

    struct XorShift32 {
        uint32_t state;
        explicit XorShift32(uint32_t seed = pros::micros())
            : state(seed == 0 ? 0x12345678u : seed) {}
        inline uint32_t next_u32() {
            uint32_t x = state;
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            return state = x;
        }
        inline float next_f32()  { return (next_u32() >> 8) * (1.0f / (1u << 24)); }
        inline float uniform(float lo, float hi) { return lo + next_f32() * (hi - lo); }
        inline float gaussian(float std_dev) {
            const float u1 = std::max(next_f32(), 1e-12f);
            const float u2 = next_f32();
            return std_dev * std::sqrt(-2.0f * std::log(u1))
                           * std::cos(2.0f * (float)M_PI * u2);
        }
    } rng;

    inline float degToRad(float d) { return d * (float)M_PI / 180.0f; }
    inline float radToDeg(float r) { return r * 180.0f / (float)M_PI; }
    inline float wrapAngle(float a) {
        a = std::fmod(a + 180.0f, 360.0f);
        if (a < 0.0f) a += 360.0f;
        return a - 180.0f;
    }

    void StartMCL(double x, double y) {
        std::lock_guard<pros::Mutex> lock(particle_mutex);
        rng = XorShift32(pros::micros());
        
        lemlib::Pose currOdom = chassis.getPose();
        
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i] = std::clamp<float>(
                (float)x + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_y[i] = std::clamp<float>(
                (float)y + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            
            particle_theta[i] = wrapAngle((float)currOdom.theta + rng.gaussian(0.2f));
            particle_weights[i] = 1.0f / NUM_PARTICLES;
        }
        w_slow = w_fast = 1.0f / NUM_PARTICLES;
        global_X = x;
        global_Y = y;
        global_Theta = currOdom.theta;
        global_Confidence = 1.0f;
    }

    void MotionUpdate(double dX_global, double dY_global, double dTheta) {
        const float dist = (float)std::hypot(dX_global, dY_global);
        const float turn_factor = std::abs((float)dTheta) * 0.05f; 
        
        const float c = 1.0f - std::clamp((float)global_Confidence, 0.0f, 1.0f);
        const float transStd = (float)(PARAMS_TRANS_BASE + c * 0.3)
                             + (float)(PARAMS_TRANS_GAIN + c * 0.04) * (dist + turn_factor);

        std::lock_guard<pros::Mutex> lock(particle_mutex);
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            
            // Sweet spot noise injection allows correction without collapsing
            particle_theta[i] = wrapAngle(particle_theta[i] + (float)dTheta + rng.gaussian(0.15f + turn_factor * 0.15f));

            float math_theta_deg = 90.0f - particle_theta[i]; 
            const float theta_rad = degToRad(math_theta_deg);
            
            const float cos_t = std::cos(theta_rad);
            const float sin_t = std::sin(theta_rad);

            const float nF = rng.gaussian(transStd);
            const float nS = rng.gaussian(transStd * 0.6f);
            
            // nF and nS are local, rotated into global frame here
            const float noise_X = nF * cos_t - nS * sin_t;
            const float noise_Y = nF * sin_t + nS * cos_t;

            // Added to global displacement
            particle_x[i] = std::clamp<float>(
                particle_x[i] + (float)dX_global + noise_X,
                FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_y[i] = std::clamp<float>(
                particle_y[i] + (float)dY_global + noise_Y,
                FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
        }
    }

    // Pass the absolute IMU reading into the update function
    void SensorUpdate(const std::vector<float>& measurements, float current_confidence, float imu_theta) {
        std::lock_guard<pros::Mutex> lock(particle_mutex);
        float sum_w = 0.0f;

        const float dynamic_sensor_sig = 1.0f + (1.0f - std::clamp(current_confidence, 0.0f, 1.0f)) * 1.0f;

        for (int i = 0; i < NUM_PARTICLES; ++i) {
            float w = 1.0f;

            float math_theta_deg = 90.0f - particle_theta[i];
            const float theta_rad = degToRad(math_theta_deg);
            const float cos_t = std::cos(theta_rad);
            const float sin_t = std::sin(theta_rad);

            for (size_t s = 0; s < measurements.size(); ++s) {
                if (measurements[s] < 0.0f) continue;

                const SensorConfig& sc = SENSOR_CONFIGS[s];

                float sensor_x = particle_x[i] + sc.y * cos_t + sc.x * sin_t;
                float sensor_y = particle_y[i] + sc.y * sin_t - sc.x * cos_t;

                float absolute_lemlib_deg = particle_theta[i] + sc.angle;
                float math_beam_deg = 90.0f - absolute_lemlib_deg;
                const float beam_rad = degToRad(math_beam_deg);

                float v_x = std::cos(beam_rad);
                float v_y = std::sin(beam_rad);

                float d_x = 999.0f;
                if (v_x > 1e-4f)       d_x = (FIELD_MAX - sensor_x) / v_x;
                else if (v_x < -1e-4f) d_x = (FIELD_MIN - sensor_x) / v_x;

                float d_y = 999.0f;
                if (v_y > 1e-4f)       d_y = (FIELD_MAX - sensor_y) / v_y;
                else if (v_y < -1e-4f) d_y = (FIELD_MIN - sensor_y) / v_y;

                float expected = std::min(d_x, d_y);
                float err = measurements[s] - expected;

                float p_hit = std::exp(-0.5f * err * err / (dynamic_sensor_sig * dynamic_sensor_sig));
                float p_short = (measurements[s] < expected) ? (LAMBDA_SHORT * std::exp(-LAMBDA_SHORT * measurements[s])) : 0.0f;
                float p_rand = 1.0f / MAX_SENSOR_READING; 

                float p = (Z_HIT * p_hit) + (Z_SHORT * p_short) + (Z_RAND * p_rand);
                w *= std::max(p, 1e-6f); 
            }

            // IMU Fusion: Treat the absolute IMU reading as a measurement update
            float theta_error = wrapAngle(particle_theta[i] - imu_theta);
            float sigma_theta = 1.0f; // 1 degree of standard deviation for the IMU
            float p_theta = std::exp(-0.5f * theta_error * theta_error / (sigma_theta * sigma_theta));
            w *= std::max(p_theta, 1e-6f);

            particle_weights[i] = w;
            sum_w += w;
        }

        const float w_avg = sum_w / NUM_PARTICLES;
        if (w_slow < 1e-10f) w_slow = w_avg;
        if (w_fast < 1e-10f) w_fast = w_avg;
        w_slow += ALPHA_SLOW * (w_avg - w_slow);
        w_fast += ALPHA_FAST * (w_avg - w_fast);

        if (sum_w > 1e-10f) {
            for (int i = 0; i < NUM_PARTICLES; ++i) particle_weights[i] /= sum_w;
        } else {
            const float u = 1.0f / NUM_PARTICLES;
            for (int i = 0; i < NUM_PARTICLES; ++i) particle_weights[i] = u;
        }
    }

    float computeESS() {
        std::lock_guard<pros::Mutex> lock(particle_mutex);
        float sq = 0.0f;
        for (int i = 0; i < NUM_PARTICLES; ++i)
            sq += particle_weights[i] * particle_weights[i];
        return (sq > 1e-20f) ? 1.0f / sq : 0.0f;
    }

    void Resample() {
        std::lock_guard<pros::Mutex> lock(particle_mutex);

        const float ratio = (w_slow > 1e-10f) ? (w_fast / w_slow) : 1.0f;
        const float inject_rate = std::clamp(1.0f - ratio, 0.0f, 0.30f); 
        const int num_inject = (int)(NUM_PARTICLES * inject_rate);
        const int num_keep = NUM_PARTICLES - num_inject;

        static float new_x[NUM_PARTICLES];
        static float new_y[NUM_PARTICLES];
        static float new_theta[NUM_PARTICLES];

        float step = 1.0f / (num_keep > 0 ? num_keep : 1);
        float r   = rng.next_f32() * step;
        float cum = particle_weights[0];
        int   j   = 0;

        for (int m = 0; m < num_keep; ++m) {
            float u = r + (float)m * step;
            while (u > cum && j < NUM_PARTICLES - 1) {
                cum += particle_weights[++j];
            }
            new_x[m] = particle_x[j];
            new_y[m] = particle_y[j];
            new_theta[m] = particle_theta[j];
        }

        for (int m = num_keep; m < NUM_PARTICLES; ++m) {
            new_x[m] = std::clamp<float>(global_X + rng.gaussian(1.5f), FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
            new_y[m] = std::clamp<float>(global_Y + rng.gaussian(1.5f), FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
            new_theta[m] = wrapAngle(global_Theta + rng.gaussian(0.5f)); 
        }

        const float uniform_w = 1.0f / NUM_PARTICLES;
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i]       = new_x[i];
            particle_y[i]       = new_y[i];
            particle_theta[i]   = new_theta[i];
            particle_weights[i] = uniform_w;
        }
    }

    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        uint32_t now = pros::millis();
        
        int print_counter = 0;
        bool first_run = true; 
        
        float mcl_std_dev = 999.0f;
        float cluster_weight_ratio = 1.0f; 
        float ess = NUM_PARTICLES;

        while (true) {
            lemlib::Pose currOdom = chassis.getPose();

            const double dX_global = currOdom.x - prevOdom.x;
            const double dY_global = currOdom.y - prevOdom.y;
            const double dTheta    = wrapAngle((float)(currOdom.theta - prevOdom.theta));

            std::vector<float> measurements(4, -1.0f);
            bool has_valid_reading = false;

            auto try_read_sensor = [&](auto& sensor, int index) {
                float val = sensor.get() / 25.4f; 
                if (val > 2.0f && val < MAX_SENSOR_READING) { 
                    measurements[index] = val; 
                    has_valid_reading = true; 
                }
            };

            try_read_sensor(frontDistance, 0);
            try_read_sensor(rightDistance, 1);
            try_read_sensor(backDistance, 2);
            try_read_sensor(leftDistance, 3);

            bool particles_updated = false;
            if (std::abs(dX_global) > 0.001 || std::abs(dY_global) > 0.001 || std::abs(dTheta) > 0.1) {
                MotionUpdate(dX_global, dY_global, dTheta);
                if (has_valid_reading) {
                    // Pass the absolute IMU reading for the measurement update
                    SensorUpdate(measurements, global_Confidence, currOdom.theta);
                }
                particles_updated = true;
            }

            if (particles_updated) {
                std::lock_guard<pros::Mutex> lock(particle_mutex);
                
                float max_w = -1.0f;
                int best_idx = 0;
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    if (particle_weights[i] > max_w) {
                        max_w = particle_weights[i];
                        best_idx = i;
                    }
                }
                
                float best_x = particle_x[best_idx];
                float best_y = particle_y[best_idx];

                float sumX = 0.0f, sumY = 0.0f, sumW = 0.0f;
                float sumX2 = 0.0f, sumY2 = 0.0f; 
                
                float sumSin = 0.0f, sumCos = 0.0f;

                const float CLUSTER_RADIUS = 12.0f; 
                
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    float dx = particle_x[i] - best_x;
                    float dy = particle_y[i] - best_y;
                    
                    if ((dx * dx + dy * dy) <= (CLUSTER_RADIUS * CLUSTER_RADIUS)) {
                        float w = particle_weights[i];
                        sumX += w * particle_x[i];
                        sumY += w * particle_y[i];
                        sumX2 += w * particle_x[i] * particle_x[i];
                        sumY2 += w * particle_y[i] * particle_y[i];
                        
                        float rad = degToRad(particle_theta[i]);
                        sumSin += w * std::sin(rad);
                        sumCos += w * std::cos(rad);

                        sumW += w;
                    }
                }

                float raw_X = best_x;
                float raw_Y = best_y;
                float raw_Theta = particle_theta[best_idx];
                
                if (sumW > 1e-6f) {
                    raw_X = sumX / sumW;
                    raw_Y = sumY / sumW;
                    raw_Theta = wrapAngle(radToDeg(std::atan2(sumSin, sumCos)));

                    float meanX = raw_X;
                    float meanY = raw_Y;
                    float varX = (sumX2 / sumW) - (meanX * meanX);
                    float varY = (sumY2 / sumW) - (meanY * meanY);
                    
                    mcl_std_dev = std::sqrt(std::max(0.0f, varX + varY));
                    cluster_weight_ratio = sumW; 
                }

                const float EMA_ALPHA = 0.20f; 
                
                if (first_run) {
                    global_X = currOdom.x; 
                    global_Y = currOdom.y;
                    global_Theta = currOdom.theta;
                    first_run = false;
                } else {
                    global_X = global_X + EMA_ALPHA * (raw_X - global_X);
                    global_Y = global_Y + EMA_ALPHA * (raw_Y - global_Y);
                    global_Theta = wrapAngle(global_Theta + EMA_ALPHA * wrapAngle(raw_Theta - global_Theta));
                }

                // Confidence metric is now derived from spatial clustering
                global_Confidence = std::clamp(cluster_weight_ratio, 0.0f, 1.0f);
                
                ess = computeESS();
                const float ratio = (w_slow > 1e-10f) ? (w_fast / w_slow) : 1.0f;
                if (ess < NUM_PARTICLES * 0.5f || ratio < 0.9f) {
                    Resample();
                }
            }

            lemlib::Pose fusedPose(global_X, global_Y, global_Theta);
            chassis.setPose(fusedPose);

            currOdom = chassis.getPose(); 
            prevOdom = currOdom;

            if (++print_counter >= 10) {
                printf("ODOM: %.1f,%.1f,%.1f | MCL: %.1f,%.1f,%.1f | SEN: %.1f,%.1f,%.1f,%.1f | CONF: %.2f | STD: %.1f | ESS: %.0f | CLUSTER: %.2f\n",
                    currOdom.x, currOdom.y, currOdom.theta,
                    global_X, global_Y, global_Theta,
                    measurements[0], measurements[1], measurements[2], measurements[3],
                    global_Confidence, mcl_std_dev, ess, cluster_weight_ratio);
                print_counter = 0;
            }

            pros::Task::delay_until(&now, 10);
        }
    }

} // namespace MCL