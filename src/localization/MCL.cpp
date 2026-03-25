#include "MCL.h"
#include "globals.h" // Assuming this is where your chassis and sensors are actually instantiated
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
#include <cstdio>

namespace MCL {

    double PARAMS_TRANS_BASE = 0.25;   
    double PARAMS_TRANS_GAIN = 0.035;  

    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0;

    float w_slow = 0.0f, w_fast = 0.0f;
    constexpr float ALPHA_SLOW = 0.001f;
    constexpr float ALPHA_FAST = 0.1f;

    // ── Field boundary ───────────────────────────────────────────────────
    constexpr float FIELD_SIZE = 140.42f; // Could be 141.2
    constexpr float HALF_SIZE  = FIELD_SIZE * 0.5f;  // 70.21"
    constexpr float FIELD_MIN  = -HALF_SIZE;
    constexpr float FIELD_MAX  =  HALF_SIZE;

    // Reject any sensor reading beyond this (inches)
    constexpr float MAX_SENSOR_READING = 65.0f;  

    // ─────────────────────────── Particle Storage ────────────────────────
    float particle_x[NUM_PARTICLES];
    float particle_y[NUM_PARTICLES];
    float particle_weights[NUM_PARTICLES];

    pros::Mutex particle_mutex;

    // ─────────────────────────── Sensor Configuration ──────────────────
    struct SensorConfig {
        float x;      // offset Right of robot center (inches)
        float y;      // offset Forward of robot center (inches)
        float angle;  // mounting angle relative to robot forward (deg)
    };

    const std::vector<SensorConfig> SENSOR_CONFIGS = {
        { -1.75f,  2.8f,   0.0f },    // 0: front
        {  4.6f,   4.3f,  90.0f },    // 1: right
        {  1.75f, -0.6f, 180.0f },    // 2: back
        { -5.1f,   4.3f, -90.0f }     // 3: left
    };

    // ─────────────────────────── RNG ────────────────────────────────────
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

    // ─────────────────────────── Utilities ──────────────────────────────
    inline float degToRad(float d) { return d * (float)M_PI / 180.0f; }
    inline float wrapAngle(float a) {
        a = std::fmod(a + 180.0f, 360.0f);
        if (a < 0.0f) a += 360.0f;
        return a - 180.0f;
    }

    // ─────────────────────────── Initialisation ─────────────────────────
    void StartMCL(double x, double y) {
        particle_mutex.take();
        rng = XorShift32(pros::micros());
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i] = std::clamp<float>(
                (float)x + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_y[i] = std::clamp<float>(
                (float)y + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_weights[i] = 1.0f / NUM_PARTICLES;
        }
        w_slow = w_fast = 1.0f / NUM_PARTICLES;
        particle_mutex.give();
    }

    // ─────────────────────────── Motion Update ──────────────────────────
    void MotionUpdate(double dX_global, double dY_global, double dTheta, double robot_theta_deg) {
        const float dist = (float)std::hypot(dX_global, dY_global);
        const float turn_factor = std::abs((float)dTheta) * 0.05f; 
        
        const float c = 1.0f - std::clamp((float)global_Confidence, 0.0f, 1.0f);
        const float transStd = (float)(PARAMS_TRANS_BASE + c * 0.3)
                             + (float)(PARAMS_TRANS_GAIN + c * 0.04) * (dist + turn_factor);

        float math_theta_deg = 90.0f - (float)robot_theta_deg; 
        const float theta_rad = degToRad(math_theta_deg);
        
        const float cos_t = std::cos(theta_rad);
        const float sin_t = std::sin(theta_rad);

        particle_mutex.take();
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            const float nF = rng.gaussian(transStd);
            const float nS = rng.gaussian(transStd * 0.6f);
            
            const float noise_X = nF * cos_t - nS * sin_t;
            const float noise_Y = nF * sin_t + nS * cos_t;

            particle_x[i] = std::clamp<float>(
                particle_x[i] + (float)dX_global + noise_X,
                FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_y[i] = std::clamp<float>(
                particle_y[i] + (float)dY_global + noise_Y,
                FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
        }
        particle_mutex.give();
    }

    // ─────────────────────────── Dynamic Raycast Sensor Update ──────────
    void SensorUpdate(const std::vector<float>& measurements, float robot_theta_deg, float current_confidence) {
        particle_mutex.take();
        float sum_w = 0.0f;

        float math_theta_deg = 90.0f - robot_theta_deg;
        const float theta_rad = degToRad(math_theta_deg);
        const float cos_t = std::cos(theta_rad);
        const float sin_t = std::sin(theta_rad);

        const float dynamic_sensor_sig = 1.5f + (1.0f - std::clamp(current_confidence, 0.0f, 1.0f)) * 4.0f;
        const float dynamic_margin = dynamic_sensor_sig * 3.0f; 

        for (int i = 0; i < NUM_PARTICLES; ++i) {
            float w = 1.0f;

            for (size_t s = 0; s < measurements.size(); ++s) {
                if (measurements[s] < 0.0f) continue;

                const SensorConfig& sc = SENSOR_CONFIGS[s];

                float sensor_x = particle_x[i] + sc.y * cos_t + sc.x * sin_t;
                float sensor_y = particle_y[i] + sc.y * sin_t - sc.x * cos_t;

                float absolute_lemlib_deg = robot_theta_deg + sc.angle;
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

                if (err > dynamic_margin) {
                    w *= 0.001f; 
                } else if (err < -dynamic_margin) {
                    w *= 0.4f;   
                } else {
                    w *= std::exp(-0.5f * err * err / (dynamic_sensor_sig * dynamic_sensor_sig));
                }
            }

            particle_weights[i] = w;
            sum_w += w;
        }

        const float w_avg = sum_w / NUM_PARTICLES;
        if (w_slow < 1e-10f) w_slow = w_avg;
        if (w_fast < 1e-10f) w_fast = w_avg;
        w_slow += ALPHA_SLOW * (w_avg - w_slow);
        w_fast += ALPHA_FAST * (w_avg - w_fast);

        if (sum_w > 1e-10f) {
            for (int i = 0; i < NUM_PARTICLES; ++i)
                particle_weights[i] /= sum_w;
        } else {
            const float u = 1.0f / NUM_PARTICLES;
            for (int i = 0; i < NUM_PARTICLES; ++i)
                particle_weights[i] = u;
        }

        particle_mutex.give();
    }

    // ─────────────────────────── ESS & Resampling ────────────────────────
    float computeESS() {
        particle_mutex.take();
        float sq = 0.0f;
        for (int i = 0; i < NUM_PARTICLES; ++i)
            sq += particle_weights[i] * particle_weights[i];
        particle_mutex.give();
        return (sq > 1e-20f) ? 1.0f / sq : 0.0f;
    }

    void Resample() {
        particle_mutex.take();

        const float ratio = (w_slow > 1e-10f) ? (w_fast / w_slow) : 1.0f;
        const float inject_rate = std::clamp(1.0f - ratio, 0.0f, 0.20f); 
        const int num_inject = (int)(NUM_PARTICLES * inject_rate);
        const int num_keep = NUM_PARTICLES - num_inject;

        static float new_x[NUM_PARTICLES];
        static float new_y[NUM_PARTICLES];

        float step = 1.0f / (num_keep > 0 ? num_keep : 1);
        float r   = rng.next_f32() * step;
        float cum = particle_weights[0];
        int   j   = 0;

        // Keep the high-performing particles
        for (int m = 0; m < num_keep; ++m) {
            float u = r + (float)m * step;
            while (u > cum && j < NUM_PARTICLES - 1) {
                cum += particle_weights[++j];
            }
            new_x[m] = particle_x[j];
            new_y[m] = particle_y[j];
        }

        // Inject new particles locally around our best guess in a tight cluster (1.5" std dev)
        for (int m = num_keep; m < NUM_PARTICLES; ++m) {
            new_x[m] = std::clamp<float>(global_X + rng.gaussian(1.5f), FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
            new_y[m] = std::clamp<float>(global_Y + rng.gaussian(1.5f), FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
        }

        const float uniform_w = 1.0f / NUM_PARTICLES;
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i]       = new_x[i];
            particle_y[i]       = new_y[i];
            particle_weights[i] = uniform_w;
        }
        
        particle_mutex.give();
    }

    // ─────────────────────────── Main Loop ──────────────────────────────
    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        uint32_t now = pros::millis();
        
        int print_counter = 0;
        bool first_run = true; // Added for the EMA filter

        while (true) {
            lemlib::Pose currOdom = chassis.getPose();

            const double dX_global = currOdom.x - prevOdom.x;
            const double dY_global = currOdom.y - prevOdom.y;
            const double dTheta    = wrapAngle((float)(currOdom.theta - prevOdom.theta));

            if (std::abs(dX_global) > 0.001 || std::abs(dY_global) > 0.001 || std::abs(dTheta) > 0.1) {

                MotionUpdate(dX_global, dY_global, dTheta, currOdom.theta);

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
                
                if (has_valid_reading) {
                    SensorUpdate(measurements, currOdom.theta, global_Confidence);
                }

                particle_mutex.take();
                
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
                const float CLUSTER_RADIUS = 15.0f;
                
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    float dx = particle_x[i] - best_x;
                    float dy = particle_y[i] - best_y;
                    
                    if ((dx * dx + dy * dy) <= (CLUSTER_RADIUS * CLUSTER_RADIUS)) {
                        float w = particle_weights[i];
                        sumX += w * particle_x[i];
                        sumY += w * particle_y[i];
                        sumX2 += w * particle_x[i] * particle_x[i];
                        sumY2 += w * particle_y[i] * particle_y[i];
                        sumW += w;
                    }
                }
                
                float mcl_std_dev = 999.0f;
                float cluster_weight_ratio = 0.0f; 

                // --- RAW CLUSTER CALCULATION ---
                float raw_X = best_x;
                float raw_Y = best_y;
                
                if (sumW > 1e-6f) {
                    raw_X = sumX / sumW;
                    raw_Y = sumY / sumW;
                    
                    float meanX = raw_X;
                    float meanY = raw_Y;
                    float varX = (sumX2 / sumW) - (meanX * meanX);
                    float varY = (sumY2 / sumW) - (meanY * meanY);
                    mcl_std_dev = std::sqrt(std::max(0.0f, varX + varY));
                    
                    cluster_weight_ratio = sumW; 
                }

                // --- EXPONENTIAL MOVING AVERAGE (EMA) FILTER ---
                const float EMA_ALPHA = 0.20f; 
                
                if (first_run) {
                    global_X = raw_X;
                    global_Y = raw_Y;
                    first_run = false;
                } else {
                    global_X = global_X + EMA_ALPHA * (raw_X - global_X);
                    global_Y = global_Y + EMA_ALPHA * (raw_Y - global_Y);
                }
                // -----------------------------------------------

                global_Theta      = currOdom.theta; 
                global_Confidence = (w_slow > 1e-10f) ? std::min(w_fast / w_slow, 1.0f) : 0.0f;
                
                particle_mutex.give();

                if (++print_counter >= 3) {
                    printf("MCL: %.2f %.2f | ODOM: %.2f %.2f | CONF: %.2f | STD: %.2f | RATIO: %.2f\n",
                        global_X, global_Y, currOdom.x, currOdom.y, global_Confidence, mcl_std_dev, cluster_weight_ratio);
                    print_counter = 0;
                }

                // ────────────────────────────────────────────────────────────
                // CONDITIONAL LEMLIB CORRECTION
                // ────────────────────────────────────────────────────────────
                if (global_Confidence > 0.85 && cluster_weight_ratio > 0.70f && 
                    std::abs(dX_global) < 2 && std::abs(dY_global) < 2) {
                    
                    double diff_x = global_X - currOdom.x;
                    double diff_y = global_Y - currOdom.y;
                    double correction_mag = std::hypot(diff_x, diff_y);
                    
                    if (correction_mag > 1.0 && correction_mag < 12.0) {
                        lemlib::Pose fusedPose(global_X, global_Y, currOdom.theta);
                        chassis.setPose(fusedPose);
                        currOdom = chassis.getPose(); 
                    }
                }

                const float ess   = computeESS();
                const float ratio = (w_slow > 1e-10f) ? (w_fast / w_slow) : 1.0f;
                if (ess < NUM_PARTICLES * 0.5f || ratio < 0.9f) {
                    Resample();
                }
            }

            prevOdom = currOdom;
            pros::Task::delay_until(&now, 10);
        }
    }

} // namespace MCL