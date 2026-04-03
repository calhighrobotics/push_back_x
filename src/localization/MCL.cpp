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

    // DEFINITIONS (Matches extern declarations in MCL.h)
    double PARAMS_TRANS_BASE = 0.25;   
    double PARAMS_TRANS_GAIN = 0.035;  

    // ── SENSOR CONFIGURATION ─────────────────────────────────────────────
    struct SensorConfig {
        float x;      // offset Right of robot center (inches)
        float y;      // offset Forward of robot center (inches)
        float angle;  // mounting angle relative to robot forward (deg)
    };

    const std::vector<SensorConfig> SENSOR_CONFIGS = {
        { -1.75f,  2.8f,   0.0f },   // 0: front
        {  4.0f,   4.3f,  90.0f },   // 1: right
        {  1.75f, -0.6f, 180.0f },   // 2: back
        { -4.0f,   4.3f, -90.0f }    // 3: left
    };

    // ── INTRINSIC MODEL PARAMETERS (Probabilistic Robotics Table 6.2) ────
    struct IntrinsicParams {
        float z_hit   = 0.85f;     
        float z_short = 0.05f;     
        float z_max   = 0.05f;     
        float z_rand  = 0.05f;     
        float sigma_hit = 2.0f;    
        float lambda_short = 0.1f; 
    } params;

    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0;

    float w_slow = 0.0f, w_fast = 0.0f;
    constexpr float ALPHA_SLOW = 0.001f;
    constexpr float ALPHA_FAST = 0.1f;

    constexpr float FIELD_SIZE = 140.42f;
    constexpr float HALF_SIZE  = FIELD_SIZE * 0.5f;
    constexpr float FIELD_MIN  = -HALF_SIZE;
    constexpr float FIELD_MAX  =  HALF_SIZE;
    constexpr float MAX_SENSOR_READING = 65.0f;  

    float particle_x[NUM_PARTICLES];
    float particle_y[NUM_PARTICLES];
    float particle_weights[NUM_PARTICLES];

    pros::Mutex particle_mutex;

    // ── RNG ─────────────────────────────────────────────────────────────
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
            return std_dev * std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * (float)M_PI * u2);
        }
    } rng;

    // ── Utilities ───────────────────────────────────────────────────────
    inline float degToRad(float d) { return d * (float)M_PI / 180.0f; }
    inline float wrapAngle(float a) {
        a = std::fmod(a + 180.0f, 360.0f);
        if (a < 0.0f) a += 360.0f;
        return a - 180.0f;
    }

    void StartMCL(double x, double y) {
        particle_mutex.take();
        rng = XorShift32(pros::micros());
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i] = std::clamp<float>((float)x + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_y[i] = std::clamp<float>((float)y + rng.gaussian(2.0f), FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_weights[i] = 1.0f / NUM_PARTICLES;
        }
        w_slow = w_fast = 1.0f / NUM_PARTICLES;
        particle_mutex.give();
    }

    void MotionUpdate(double dX_global, double dY_global, double robot_theta_deg) {
        const float dist = (float)std::hypot(dX_global, dY_global);
        const float c    = 1.0f - std::clamp((float)global_Confidence, 0.0f, 1.0f);
        const float transStd = (float)(PARAMS_TRANS_BASE + c * 0.3) + (float)(PARAMS_TRANS_GAIN + c * 0.04) * dist;

        const float theta_rad = degToRad(90.0f - (float)robot_theta_deg);
        const float cos_t = std::cos(theta_rad), sin_t = std::sin(theta_rad);

        particle_mutex.take();
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            const float nF = rng.gaussian(transStd), nS = rng.gaussian(transStd * 0.6f);
            const float noise_X = nF * cos_t - nS * sin_t, noise_Y = nF * sin_t + nS * cos_t;

            particle_x[i] = std::clamp<float>(particle_x[i] + (float)dX_global + noise_X, FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
            particle_y[i] = std::clamp<float>(particle_y[i] + (float)dY_global + noise_Y, FIELD_MIN + 0.1f, FIELD_MAX - 0.1f);
        }
        particle_mutex.give();
    }

    // ── ADVANCED BEAM MODEL (Probabilistic Robotics Table 6.1) ───────────
    void SensorUpdate(const std::vector<float>& measurements, float robot_theta_deg, float current_confidence) {
        particle_mutex.take();
        float sum_w = 0.0f;
        const float theta_rad = degToRad(90.0f - robot_theta_deg);
        const float cos_t = std::cos(theta_rad), sin_t = std::sin(theta_rad);

        for (int i = 0; i < NUM_PARTICLES; ++i) {
            float q = 1.0f; 
            for (size_t k = 0; k < measurements.size(); ++k) {
                if (measurements[k] < 0.0f) continue;

                const SensorConfig& sc = SENSOR_CONFIGS[k];
                float s_x = particle_x[i] + sc.y * cos_t + sc.x * sin_t;
                float s_y = particle_y[i] + sc.y * sin_t - sc.x * cos_t;
                float beam_rad = degToRad(90.0f - (robot_theta_deg + sc.angle));
                
                float v_x = std::cos(beam_rad), v_y = std::sin(beam_rad);
                float d_x = (v_x > 0) ? (FIELD_MAX - s_x)/v_x : (v_x < 0) ? (FIELD_MIN - s_x)/v_x : 999.0f;
                float d_y = (v_y > 0) ? (FIELD_MAX - s_y)/v_y : (v_y < 0) ? (FIELD_MIN - s_y)/v_y : 999.0f;
                float z_star = std::min(d_x, d_y);
                float z_k = measurements[k];
                float p = 0.0f;

                // p_hit (Normal distribution noise)
                if (z_k >= 0 && z_k <= MAX_SENSOR_READING) {
                    float exponent = -0.5f * std::pow((z_k - z_star) / params.sigma_hit, 2);
                    p += params.z_hit * (1.0f / (params.sigma_hit * std::sqrt(2.0f * M_PI))) * std::exp(exponent);
                }
                // p_short (Exponential distribution for dynamic obstacles)
                if (z_k >= 0 && z_k <= z_star) {
                    float eta = 1.0f / (1.0f - std::exp(-params.lambda_short * z_star));
                    p += params.z_short * eta * params.lambda_short * std::exp(-params.lambda_short * z_k);
                }
                // p_max (Measurement failure / out of range)
                if (std::abs(z_k - MAX_SENSOR_READING) < 1.0f) p += params.z_max;
                // p_rand (Uniform random noise)
                if (z_k >= 0 && z_k < MAX_SENSOR_READING) p += params.z_rand * (1.0f / MAX_SENSOR_READING);
                
                q *= p;
            }
            particle_weights[i] = q;
            sum_w += q;
        }

        const float w_avg = sum_w / NUM_PARTICLES;
        w_slow += ALPHA_SLOW * (w_avg - w_slow);
        w_fast += ALPHA_FAST * (w_avg - w_fast);

        if (sum_w > 1e-10f) {
            for (int i = 0; i < NUM_PARTICLES; ++i) particle_weights[i] /= sum_w;
        } else {
            for (int i = 0; i < NUM_PARTICLES; ++i) particle_weights[i] = 1.0f / NUM_PARTICLES;
        }
        particle_mutex.give();
    }

    float computeESS() {
        particle_mutex.take();
        float sq = 0.0f;
        for (int i = 0; i < NUM_PARTICLES; ++i) sq += particle_weights[i] * particle_weights[i];
        particle_mutex.give();
        return (sq > 1e-20f) ? 1.0f / sq : 0.0f;
    }

    void Resample() {
        particle_mutex.take();
        const float ratio = (w_slow > 1e-10f) ? (w_fast / w_slow) : 1.0f;
        const float inject_rate = std::clamp(1.0f - ratio, 0.0f, 0.20f); 
        const int num_inject = (int)(NUM_PARTICLES * inject_rate), num_keep = NUM_PARTICLES - num_inject;

        static float new_x[NUM_PARTICLES], new_y[NUM_PARTICLES];
        float step = 1.0f / (num_keep > 0 ? num_keep : 1), r = rng.next_f32() * step, cum = particle_weights[0];
        int j = 0;

        // Keep the high-performing particles
        for (int m = 0; m < num_keep; ++m) {
            float u = r + (float)m * step;
            while (u > cum && j < NUM_PARTICLES - 1) cum += particle_weights[++j];
            new_x[m] = particle_x[j]; new_y[m] = particle_y[j];
        }
        for (int m = num_keep; m < NUM_PARTICLES; ++m) {
            new_x[m] = std::clamp<float>(global_X + rng.gaussian(1.5f), FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
            new_y[m] = std::clamp<float>(global_Y + rng.gaussian(1.5f), FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
        }
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i] = new_x[i]; particle_y[i] = new_y[i];
            particle_weights[i] = 1.0f / NUM_PARTICLES;
        }
        particle_mutex.give();
    }

    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        uint32_t now = pros::millis();

        while (true) {
            lemlib::Pose currOdom = chassis.getPose();
            const double dX_global = currOdom.x - prevOdom.x, dY_global = currOdom.y - prevOdom.y;
            const double dTheta = wrapAngle((float)(currOdom.theta - prevOdom.theta));

            if (std::abs(dX_global) > 0.001 || std::abs(dY_global) > 0.001 || std::abs(dTheta) > 0.1) {
                MotionUpdate(dX_global, dY_global, currOdom.theta);
                std::vector<float> measurements(4, -1.0f);
                bool has_valid = false;
                auto try_read = [&](auto& s, int idx) {
                    float val = s.get() / 25.4f;
                    if (val > 2.0f && val < MAX_SENSOR_READING) { measurements[idx] = val; has_valid = true; }
                };
                try_read(frontDistance, 0); try_read(rightDistance, 1); try_read(backDistance, 2); try_read(leftDistance, 3);
                
                if (has_valid) SensorUpdate(measurements, currOdom.theta, global_Confidence);
                if (computeESS() < NUM_PARTICLES * 0.5f || (w_slow > 1e-10f && (w_fast/w_slow) < 0.9f)) Resample();

                particle_mutex.take();
                float max_w = -1.0f; int best_idx = 0;
                for (int i = 0; i < NUM_PARTICLES; ++i) if (particle_weights[i] > max_w) { max_w = particle_weights[i]; best_idx = i; }
                
                float sumX = 0, sumY = 0, sumW = 0, sumX2 = 0, sumY2 = 0;
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    float dx = particle_x[i] - particle_x[best_idx], dy = particle_y[i] - particle_y[best_idx];
                    if ((dx*dx + dy*dy) <= 225.0f) {
                        float w = particle_weights[i]; sumX += w*particle_x[i]; sumY += w*particle_y[i];
                        sumX2 += w*particle_x[i]*particle_x[i]; sumY2 += w*particle_y[i]*particle_y[i]; sumW += w;
                    }
                }
                
                if (sumW > 1e-6f) {
                    global_X = sumX / sumW; global_Y = sumY / sumW;
                    float var = (sumX2/sumW - (float)global_X*(float)global_X) + (sumY2/sumW - (float)global_Y*(float)global_Y);
                    float std_dev = std::sqrt(std::max(0.0f, var));
                    global_Confidence = (w_slow > 1e-10f) ? std::min(w_fast/w_slow, 1.0f) : 0.0f;

                    if (global_Confidence > 0.3 && sumW > 0.30f) {
                        double alpha = std::clamp(0.10 * global_Confidence * (2.0/(std_dev+1e-3)), 0.0, 0.25);
                        float targetX = static_cast<float>(currOdom.x + (global_X - currOdom.x)*alpha);
                        float targetY = static_cast<float>(currOdom.y + (global_Y - currOdom.y)*alpha);
                        chassis.setPose({targetX, targetY, static_cast<float>(currOdom.theta)});
                        currOdom = chassis.getPose();
                    }
                }
                particle_mutex.give();
            }
            prevOdom = currOdom;
            pros::Task::delay_until(&now, 10);
        }
    }
}
