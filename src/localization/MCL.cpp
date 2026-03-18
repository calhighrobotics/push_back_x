#include "MCL.h"
#include "globals.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
 
namespace MCL {
 
    double PARAMS_TRANS_BASE = 0.25;   // base translational noise (inches)
    double PARAMS_TRANS_GAIN = 0.025;  // noise added per inch of travel
 
    double global_X = 0, global_Y = 0, global_Theta = 0, global_Confidence = 0;
 
    float w_slow = 0.0f, w_fast = 0.0f;
    constexpr float ALPHA_SLOW = 0.001f;
    constexpr float ALPHA_FAST = 0.1f;
 
    // ── Field boundary ───────────────────────────────────────────────────
    // VEX Push Back inner playing surface: 6 tiles × 600mm = 3600mm = 141.73"
    // HALF_SIZE = 70.87" = ±1800mm (matches the VEX GPS coordinate range).
    //
    // FIELD_SIZE clamps particle positions to where the robot can physically
    // exist. It is NOT sensor range and NOT the outer wall dimension (144").
    // The original code used 140.42" (slightly off); 144" is the outer
    // wall-to-wall dimension and is also wrong for this purpose.
    constexpr float FIELD_SIZE = 141.73f;
    constexpr float HALF_SIZE  = FIELD_SIZE * 0.5f;  // 70.87"
    constexpr float FIELD_MIN  = -HALF_SIZE;
    constexpr float FIELD_MAX  =  HALF_SIZE;
 
    // Reject any sensor reading beyond this. The field diagonal is
    // sqrt(2) × 70.87 ≈ 100.2". Anything larger is noise or open-air.
    constexpr float MAX_SENSOR_READING = 105.0f;  // inches
 
    // ─────────────────────────── 2-State Particle Storage ──────────────
    // THIS IS A 2-STATE MCL: particles only carry (x, y).
    // Heading is never estimated by MCL — it is taken directly from the IMU
    // via chassis.getPose().theta, which is far more accurate than anything
    // 4 distance sensors could infer.
    float particle_x[NUM_PARTICLES];
    float particle_y[NUM_PARTICLES];
    float particle_weights[NUM_PARTICLES];
 
    pros::Mutex particle_mutex;
 
    // ─────────────────────────── Landmarks ─────────────────────────────
    // VEX Push Back 2025-26  —  goalposts and match-loader posts only.
    //
    // Coordinates confirmed by user. Origin at field centre, LemLib inches.
    // Goal posts at (±23, ±47.5), match loaders at (±68, ±47).
    struct Landmark { float x, y, std_dev; };
 
    constexpr float WALL      = HALF_SIZE;          // 70.87" — perimeter
    // Confirmed field coordinates (all 4 quadrants, origin at centre).
    // Goal posts at (±23, ±47.5), match loaders at (±68, ±47).
    const Landmark LANDMARKS[] = {
        // ── Goal posts — (±23, ±47.5) ───────────────────────────────────
        { -23.0f,  47.5f, 2.5f },
        {  23.0f,  47.5f, 2.5f },
        { -23.0f, -47.5f, 2.5f },
        {  23.0f, -47.5f, 2.5f },
 
        // ── Match loaders — (±68, ±47) ───────────────────────────────────
        { -68.0f,  47.0f, 3.0f },
        {  68.0f,  47.0f, 3.0f },
        { -68.0f, -47.0f, 3.0f },
        {  68.0f, -47.0f, 3.0f },
    };
    constexpr int NUM_LANDMARKS = sizeof(LANDMARKS) / sizeof(LANDMARKS[0]);
 
    // A sensor beam matches a landmark only if the beam points within this
    // angular window of the landmark direction.
    constexpr float SENSOR_FOV_HALF_DEG = 15.0f;
 
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
    inline float radToDeg(float r) { return r * 180.0f / (float)M_PI; }
    inline float wrapAngle(float a) {
        a = std::fmod(a + 180.0f, 360.0f);
        if (a < 0.0f) a += 360.0f;
        return a - 180.0f;
    }
 
    // ─────────────────────────── Initialisation ─────────────────────────
    // No theta parameter — heading belongs to the IMU, not MCL.
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
    // 2-state: perturbs (x, y) only.
    // robot_theta_deg is the IMU heading, used only to rotate the noise
    // vector into the global frame (forward vs. lateral noise split).
    void MotionUpdate(double dX_global, double dY_global,
                      double robot_theta_deg) {
        const float dist = (float)std::hypot(dX_global, dY_global);
        const float c    = 1.0f - std::clamp((float)global_Confidence, 0.0f, 1.0f);
        const float transStd = (float)(PARAMS_TRANS_BASE + c * 0.3)
                             + (float)(PARAMS_TRANS_GAIN + c * 0.04) * dist;
 
        const float theta_rad = degToRad((float)robot_theta_deg);
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
 
    // ─────────────────────────── Sensor Update ──────────────────────────
    // 2-state: beam directions use global_Theta (from IMU/odometry),
    // NOT per-particle heading.  This is correct by design — heading is
    // not a state MCL needs to estimate here.
    //
    // measurements : sensor readings in inches, already filtered for range
    // angles_deg   : sensor mounting angle relative to robot forward (+x)
    void SensorUpdate(const std::vector<float>& measurements,
                      const std::vector<float>& angles_deg) {
        particle_mutex.take();
        float sum_w = 0.0f;
 
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            float w = 1.0f;
 
            for (size_t s = 0; s < measurements.size(); ++s) {
                // Beam direction in the world frame using the IMU heading
                const float beam_rad =
                    degToRad((float)global_Theta + angles_deg[s]);
 
                float best = 0.0001f;
 
                for (int k = 0; k < NUM_LANDMARKS; ++k) {
                    const float dx = LANDMARKS[k].x - particle_x[i];
                    const float dy = LANDMARKS[k].y - particle_y[i];
 
                    const float lm_rad      = std::atan2(dy, dx);
                    const float ang_diff    = std::abs(
                        wrapAngle(radToDeg(lm_rad - beam_rad)));
 
                    if (ang_diff > SENSOR_FOV_HALF_DEG) continue;
 
                    const float expected = std::hypot(dx, dy);
                    const float err      = measurements[s] - expected;
                    const float sig      = LANDMARKS[k].std_dev;
                    const float p        = std::exp(
                        -0.5f * err * err / (sig * sig));
                    if (p > best) best = p;
                }
                w *= best;
            }
 
            particle_weights[i] = w;
            sum_w += w;
        }
 
        // ── Slow/fast averages computed BEFORE normalisation ─────────────
        // After normalisation weights sum to 1 so w_avg = 1/N always.
        // That ordering bug was present in the original code and silently
        // broke AMCL recovery.
        const float w_avg = sum_w / NUM_PARTICLES;
        if (w_slow < 1e-10f) w_slow = w_avg;
        if (w_fast < 1e-10f) w_fast = w_avg;
        w_slow += ALPHA_SLOW * (w_avg - w_slow);
        w_fast += ALPHA_FAST * (w_avg - w_fast);
 
        // ── Normalise ────────────────────────────────────────────────────
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
 
    // ─────────────────────────── ESS ────────────────────────────────────
    float computeESS() {
        particle_mutex.take();
        float sq = 0.0f;
        for (int i = 0; i < NUM_PARTICLES; ++i)
            sq += particle_weights[i] * particle_weights[i];
        particle_mutex.give();
        return (sq > 1e-20f) ? 1.0f / sq : 0.0f;
    }
 
    // ─────────────────────────── Resampling ─────────────────────────────
    // Augmented MCL: systematic resampling + random injection.
    // Static buffers: avoids heap allocation inside a PROS task every
    // resample cycle (which was a potential stack-overflow risk).
    void Resample() {
        particle_mutex.take();
 
        const float ratio  = (w_slow > 1e-10f) ? (w_fast / w_slow) : 1.0f;
        const float inject = std::clamp(1.0f - ratio, 0.0f, 0.5f);
        const float step   = 1.0f / NUM_PARTICLES;
 
        static float new_x[NUM_PARTICLES];
        static float new_y[NUM_PARTICLES];
 
        float r   = rng.next_f32() * step;
        float cum = particle_weights[0];
        int   j   = 0;
 
        for (int m = 0; m < NUM_PARTICLES; ++m) {
            if (rng.next_f32() < inject) {
                new_x[m] = rng.uniform(FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
                new_y[m] = rng.uniform(FIELD_MIN + 1.0f, FIELD_MAX - 1.0f);
            } else {
                const float u = r + (float)m * step;
                while (u > cum && j < NUM_PARTICLES - 1)
                    cum += particle_weights[++j];
                new_x[m] = particle_x[j];
                new_y[m] = particle_y[j];
            }
        }
 
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particle_x[i]       = new_x[i];
            particle_y[i]       = new_y[i];
            particle_weights[i] = step;
        }
        particle_mutex.give();
    }
 
    // ─────────────────────────── Main Loop ──────────────────────────────
    void MonteCarlo() {
        lemlib::Pose prevOdom = chassis.getPose();
        uint32_t now = pros::millis();
 
        while (true) {
            lemlib::Pose currOdom = chassis.getPose();
 
            const double dX_global = currOdom.x - prevOdom.x;
            const double dY_global = currOdom.y - prevOdom.y;
            const double dTheta    = wrapAngle(
                (float)(currOdom.theta - prevOdom.theta));
 
            if (std::abs(dX_global) > 0.001 ||
                std::abs(dY_global) > 0.001 ||
                std::abs(dTheta)    > 0.1) {
 
                MotionUpdate(dX_global, dY_global, currOdom.theta);
 
                // ── Replace with real sensor reads ─────────────────────
                std::vector<float> raw_m = {10.0f, 12.0f, 11.5f, 9.8f};
                std::vector<float> raw_a = {0.0f, 90.0f, -90.0f, 180.0f};
 
                // Pre-filter: discard out-of-range readings
                std::vector<float> good_m, good_a;
                for (size_t i = 0; i < raw_m.size(); ++i) {
                    if (raw_m[i] > 0.0f && raw_m[i] < MAX_SENSOR_READING) {
                        good_m.push_back(raw_m[i]);
                        good_a.push_back(raw_a[i]);
                    }
                }
                if (!good_m.empty()) SensorUpdate(good_m, good_a);
 
                // ── Resample trigger ───────────────────────────────────
                const float ess   = computeESS();
                const float ratio = (w_slow > 1e-10f)
                                  ? (w_fast / w_slow) : 1.0f;
                if (ess < NUM_PARTICLES * 0.5f || ratio < 0.9f)
                    Resample();
 
                // ── Weighted mean pose estimate (X, Y only) ────────────
                particle_mutex.take();
                float sumX = 0.0f, sumY = 0.0f;
                for (int i = 0; i < NUM_PARTICLES; ++i) {
                    sumX += particle_weights[i] * particle_x[i];
                    sumY += particle_weights[i] * particle_y[i];
                }
                global_X          = sumX;
                global_Y          = sumY;
                global_Theta      = currOdom.theta; // IMU owns heading
                global_Confidence = (w_slow > 1e-10f)
                                  ? std::min(w_fast / w_slow, 1.0f)
                                  : 0.0f;
                particle_mutex.give();
            }
 
            prevOdom = currOdom;
            pros::Task::delay_until(&now, 30);
        }
    }
 
} // namespace MCL