#include "globals.h"
#include "MCL.h"
#include "lemlib/util.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <random>

// Check for M_PI definition
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

namespace MCL {

    // --- TUNING CONSTANTS ---
    // Increased SIGMA to 3.0 to tolerate VEX sensor noise better
    const double SIGMA_MEASUREMENT = 3.0; 
    const double INV_VAR_DENOM = -1.0 / (2.0 * SIGMA_MEASUREMENT * SIGMA_MEASUREMENT);
    
    // Motion Model Noise
    const double MOTION_NOISE_LIN = 0.05;    // Base linear noise (inches)
    const double MOTION_NOISE_ROT = 0.01;    // Base rotational noise (radians)
    const double MOTION_NOISE_PCT = 0.08;    // Proportional noise (8% of distance moved)

    // RMCL (Augmented MCL) Recovery Parameters
    // Controls how quickly we decide we are "lost" and need to inject random particles
    const double ALPHA_SLOW = 0.05; // Long-term average weight decay
    const double ALPHA_FAST = 0.3;  // Short-term average weight decay
    
    // Internal State
    double w_slow = 0.0;
    double w_fast = 0.0;
    double X = 0, Y = 0, theta = 0;
    
    mt19937 Random(random_device{}());

    vector<Particle> Particles(num_particles);
    vector<Particle> NextGen(num_particles); 

    // SENSOR SETUP
    // Ensure offsets are accurate (inches from robot center)
    vector<MCLDistanceSensor> Sensors = {
        MCLDistanceSensor(frontDistance, Point(-3, -0.75), FRONT),
        MCLDistanceSensor(rightDistance, Point(6.3, -0.5), RIGHT),
        MCLDistanceSensor(leftDistance, Point(-6.4, -0.5), LEFT),
        MCLDistanceSensor(backDistance, Point(-3, -10.5), BACK),
    };

    Field field_;
    vector<MCLDistanceSensor> activeSensors;
    pros::Mutex particle_mutex;

    const double LOOP_DELAY_MS = 20.0;
    const double MM_TO_IN = 0.0393701; // Critical Conversion Factor

    // --- HELPER FUNCTIONS ---

    // Convert LemLib degrees (0=North, CW) to Standard Math Radians (0=East, CCW)
    double lemlibToMathRad(double lemlibDeg) {
        return M_PI_2 - lemlib::degToRad(lemlibDeg);
    }

    // Convert Standard Math Radians back to LemLib degrees
    double mathRadToLemlib(double mathRad) {
        return lemlib::radToDeg(M_PI_2 - mathRad);
    }

    // Normalize angle to [-PI, PI]
    double normalizeAngle(double rad) {
        rad = fmod(rad + M_PI, 2.0 * M_PI);
        if (rad < 0) rad += 2.0 * M_PI;
        return rad - M_PI;
    }

    // --- CORE FUNCTIONS ---

    void StartMCL(double x_, double y_, double theta_) {
        particle_mutex.take();
        
        // Resize vectors just in case
        if(Particles.size() != num_particles) Particles.resize(num_particles);
        if(NextGen.size() != num_particles) NextGen.resize(num_particles);

        // Initial distribution noise
        uniform_real_distribution<double> dist_x(-start_std_pos[0], start_std_pos[0]);
        uniform_real_distribution<double> dist_y(-start_std_pos[1], start_std_pos[1]);
        uniform_real_distribution<double> dist_t(-0.1, 0.1); 

        double startThetaMath = lemlibToMathRad(theta_);

        for (int i = 0; i < num_particles; ++i) {
            Particles[i].x = x_ + dist_x(Random);
            Particles[i].y = y_ + dist_y(Random);
            Particles[i].theta = normalizeAngle(startThetaMath + dist_t(Random));
            Particles[i].weight = 1.0 / num_particles;
        }
        
        // Reset RMCL weights so we don't inject particles immediately
        w_slow = 0.0;
        w_fast = 0.0;
        
        particle_mutex.give();
    }

    void MonteCarlo(void) {
        lemlib::Pose lastPose = chassis.getPose(true);
        
        while (true) {
            uint32_t start_time = pros::millis();

            // ==========================================
            // 1. MOTION UPDATE (ODOMETRY)
            // ==========================================
            lemlib::Pose currentPose = chassis.getPose(true);
            
            double dX = currentPose.x - lastPose.x;
            double dY = currentPose.y - lastPose.y;
            double dTheta = lemlib::degToRad(currentPose.theta - lastPose.theta);

            double prevHeadingMath = lemlibToMathRad(lastPose.theta);
            
            // Transform global delta to local robot frame
            double localFwd =  dX * cos(prevHeadingMath) + dY * sin(prevHeadingMath);
            double localStr = -dX * sin(prevHeadingMath) + dY * cos(prevHeadingMath); 

            double moveDist = std::hypot(dX, dY);
            
            // Only update particles if we actually moved (saves CPU)
            bool moved = (moveDist > 0.01 || abs(dTheta) > 0.001);

            if (moved) {
                // Adaptive noise based on movement
                normal_distribution<double> noise_lin(0, MOTION_NOISE_LIN + (moveDist * MOTION_NOISE_PCT));
                normal_distribution<double> noise_rot(0, MOTION_NOISE_ROT + (std::abs(dTheta) * 0.1));

                particle_mutex.take();
                for (auto& p : Particles) {
                    double rotNoise = noise_rot(Random);
                    
                    // Update Angle (dTheta is negative in Math frame vs LemLib frame, usually handled by unit circle logic)
                    // Since we converted to Local frame first, we just subtract dTheta
                    p.theta -= (dTheta + rotNoise); 
                    p.theta = normalizeAngle(p.theta);

                    // Add Noise to Local Move
                    double noisyFwd = localFwd + noise_lin(Random);
                    double noisyStr = localStr + noise_lin(Random);

                    // Rotate back to Global (Math Frame) using Particle's OWN theta
                    p.x += noisyFwd * cos(p.theta) - noisyStr * sin(p.theta);
                    p.y += noisyFwd * sin(p.theta) + noisyStr * cos(p.theta);
                }
                particle_mutex.give();
                lastPose = currentPose;
            }

            // ==========================================
            // 2. SENSOR UPDATE
            // ==========================================
            activeSensors.clear();
            for (auto& sensor : Sensors) {
                sensor.Measure(); // Gets raw mm from V5

                // *** CRITICAL FIX: Convert Millimeters to Inches ***
                sensor.measurement = sensor.measurement * MM_TO_IN;

                // Range Check (1 inch to ~8 feet)
                if (sensor.measurement > 1.0 && sensor.measurement < 100.0) { 
                    activeSensors.push_back(sensor);
                }
            }

            particle_mutex.take();
            
            double total_weight = 0.0;

            if (!activeSensors.empty()) {
                for (auto& p : Particles) {
                    // Bounds check - kill particles outside field
                    if (abs(p.x) > field_.HalfSize || abs(p.y) > field_.HalfSize) {
                         p.weight = 0.0;
                         continue;
                    }

                    double likelihood = 1.0;
                    
                    for (auto& sensor : activeSensors) {
                        double predicted = field_.get_sensor_distance(p, sensor);
                        double measured = sensor.measurement;

                        // FEATURE: Dynamic Obstacle Rejection
                        // If we see something >10 inches closer than the map says, it's a dynamic object (robot/goal).
                        // Ignore it so we don't mess up our position.
                        if (measured < (predicted - 10.0)) {
                            continue; 
                        }

                        // FEATURE: Max Range Penalties
                        // If map says we SHOULD see a wall (predicted < 90) 
                        // but sensor sees infinity (measured > 90), apply penalty.
                        if (predicted < 90.0 && measured > 95.0) {
                             likelihood *= 0.4; 
                             continue;
                        }

                        // Gaussian Likelihood
                        double error = predicted - measured;
                        double prob = exp((error * error) * INV_VAR_DENOM);
                        
                        // Clamp probability so a single bad reading doesn't kill the particle entirely
                        likelihood *= std::max(prob, 0.01); 
                    }

                    p.weight = likelihood;
                    total_weight += likelihood;
                }
            } else {
                // No sensors? No weight update.
                total_weight = 0.0;
            }

            // ==========================================
            // 3. RMCL RESAMPLING
            // ==========================================
            double inject_prob = 0.0;
            
            if (total_weight > 0) {
                double avg_weight = total_weight / num_particles;

                // Update running averages
                w_slow += ALPHA_SLOW * (avg_weight - w_slow);
                w_fast += ALPHA_FAST * (avg_weight - w_fast);
                
                // Calculate injection probability (High when w_fast << w_slow)
                inject_prob = std::max(0.0, 1.0 - (w_fast / w_slow));
                
                // Normalize weights
                double max_w = 0;
                for(auto& p : Particles) {
                    p.weight /= total_weight;
                    if(p.weight > max_w) max_w = p.weight;
                }

                // Low Variance Resampling with Random Injection
                uniform_real_distribution<double> dist_field(-field_.HalfSize + 2, field_.HalfSize - 2);
                uniform_real_distribution<double> dist_pi(-M_PI, M_PI);
                uniform_real_distribution<double> dist_01(0.0, 1.0);
                uniform_real_distribution<double> dist_resample(0.0, 1.0);

                double beta = 0.0;
                int index = std::uniform_int_distribution<int>(0, num_particles-1)(Random);

                for (int i = 0; i < num_particles; ++i) {
                    if (dist_01(Random) < inject_prob) {
                        // RECOVERY: Inject random particle
                        NextGen[i].x = dist_field(Random);
                        NextGen[i].y = dist_field(Random);
                        NextGen[i].theta = dist_pi(Random);
                        NextGen[i].weight = 1.0 / num_particles;
                    } 
                    else {
                        // STANDARD: Resample existing particles
                        beta += dist_resample(Random) * 2.0 * max_w;
                        while (beta > Particles[index].weight) {
                            beta -= Particles[index].weight;
                            index = (index + 1) % num_particles;
                        }
                        NextGen[i] = Particles[index];
                    }
                }
                Particles = NextGen;
            }

            // ==========================================
            // 4. CALCULATE AVERAGE POSITION
            // ==========================================
            double mean_x = 0, mean_y = 0;
            double sum_sin = 0, sum_cos = 0;
            
            for (const auto& p : Particles) {
                mean_x += p.x;
                mean_y += p.y;
                sum_sin += sin(p.theta);
                sum_cos += cos(p.theta);
            }

            X = mean_x / num_particles;
            Y = mean_y / num_particles;
            double avgThetaMath = atan2(sum_sin, sum_cos);
            
            // Output to global variables
            theta = mathRadToLemlib(avgThetaMath); 
            
            particle_mutex.give();

            // ==========================================
            // 5. DEBUGGING
            // ==========================================
            static int print_timer = 0;
            if (print_timer++ % 10 == 0) {
                double dist_err = std::hypot(currentPose.x - X, currentPose.y - Y);
                printf("Err: %.2f | MCL: (%.1f, %.1f) | Inj: %.0f%%\n", 
                      dist_err, X, Y, inject_prob * 100.0);
            }

            pros::Task::delay_until(&start_time, LOOP_DELAY_MS);
        }
    }
    
    double getAvgVelocity(void) noexcept { return 0.0; }
}