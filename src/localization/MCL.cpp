#include "globals.h"
#include <cmath>          // FIXED: <cmath> instead of "cmath"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/util.hpp"
#include "MCL.h"

using namespace std;

namespace MCL {
  double X = 0, Y = 0, theta = 0;

  mt19937 Random(random_device {} ());

  double Velo = 0;
  double Deviation;
  double weights_sum;
  double start_theta;

  vector<Particle> Particles(250); 
  vector<MCLDistanceSensor> Sensors = {
    MCLDistanceSensor(frontDistance, Point(-5.25, 0), FRONT),
    MCLDistanceSensor(rightDistance, Point(-5.25, -2.5), RIGHT),
    MCLDistanceSensor(leftDistance, Point(5.25, -2.25), LEFT),
    MCLDistanceSensor(backDistance, Point(-4.75, -7.5), BACK), 
  };

  Field field_;
  vector<MCLDistanceSensor> activeSensors;
  pros::Mutex particle_mutex;
  double getAvgVelocity(void) noexcept;
  void StartMCL(double x_, double y_, double theta_);
  void MonteCarlo(void);

  const double LOOP_DELAY_MS = 10; 
  const double LOOP_DT_SEC = LOOP_DELAY_MS / 1000.0; 

  void StartMCL(double x_, double y_, double theta_) {
    particle_mutex.take();
    
    Particles.clear();
    Particles.resize(num_particles); 

    uniform_real_distribution<double> start_dist_x(-start_std_pos[0], start_std_pos[0]);
    uniform_real_distribution<double> start_dist_y(-start_std_pos[1], start_std_pos[1]);
    uniform_real_distribution<double> start_dist_theta(-start_std_pos[2], start_std_pos[2]);

    for (int i = 0; i < num_particles; ++i) {
      Particles[i].x = x_ + start_dist_x(Random);
      Particles[i].y = y_ + start_dist_y(Random);
      Particles[i].theta = lemlib::radToDeg(theta_) + start_dist_theta(Random);
      Particles[i].weight = 1.0;
    }
    

    particle_mutex.give();
  }

  vector<Particle> Resampled(num_particles);
  vector<double> CDF(num_particles);

void MonteCarlo(void) {
  uint32_t lastSensorUpdate = 0;
  const uint32_t SENSOR_PERIOD_MS = 33; 

  // FIX 1: Ensure vector size matches the constant in your header
  if (Particles.size() != num_particles) {
      Particles.resize(num_particles);
  }

  while (true) {
    uint32_t start_time = pros::millis();

    /* =============================================
       1. PREDICTION (Runs EVERY 10ms)
       ============================================= */
    Velo = getAvgVelocity();
    double distance_step = Velo * LOOP_DT_SEC;
    
    // Safety: ensure stddev is never absolute 0
    double noise_stddev = std::max(0.0001, std::abs(distance_step * 0.25));
    normal_distribution<double> dist_pos(0, noise_stddev);

    lemlib::Pose odomPose = chassis.getPose(true);
    const double theta_ = odomPose.theta; 
    
    // Calculate heading vectors
    const double rotated_theta = M_PI_2 - theta_;
    const float cos_theta = cosf(rotated_theta);
    const float sin_theta = sinf(rotated_theta);

    particle_mutex.take();

    // Move Particles
    for (auto& p : Particles) {
      // 1. Update Heading
      p.theta = theta_; 
      
      // 2. CRITICAL FIX: Update p.step for Raycasting
      p.step.x = cos_theta;
      p.step.y = sin_theta;

      // 3. Move X/Y with noise
      p.x += (distance_step * cos_theta) + dist_pos(Random);
      p.y += (distance_step * sin_theta) + dist_pos(Random);

      // 4. Clamp to field bounds
      p.x = std::clamp(p.x, -field_.HalfSize, field_.HalfSize);
      p.y = std::clamp(p.y, -field_.HalfSize, field_.HalfSize);
    }

    /* =============================================
       2. SENSOR UPDATE (~35 ms) WITH ESS
       ============================================= */
    bool doSensorUpdate = (pros::millis() - lastSensorUpdate) >= SENSOR_PERIOD_MS;

    if (doSensorUpdate) {
      lastSensorUpdate = pros::millis();

      // Filter active sensors
      activeSensors.clear();
      for (auto& sensor : Sensors) {
        sensor.Measure();
        if (sensor.measurement > 20 && sensor.measurement < 2000) {
          activeSensors.push_back(sensor);
        }
      }

      if (!activeSensors.empty()) {
          weights_sum = 0.0;
          
          // --- A. Update Weights (ACCUMULATIVE) ---
          for (auto& P : Particles) {
            double wt = P.weight; 
            
            for (auto& sensor : activeSensors) {
              const double predicted = field_.get_sensor_distance(P, sensor);
              
              double likelihood = 1e-9; 
              // 10000 is an arbitrary "max field distance" check
              if (predicted >= 0 && predicted < 10000) { 
                 Deviation = predicted - sensor.measurement;
                 likelihood = exp(-(Deviation * Deviation) * inv_varience) * inv_base;
              }
              
              likelihood = std::max(likelihood, 1e-9);
              wt *= likelihood;
            }
            P.weight = wt;
            weights_sum += wt;
          }

          // --- B. Normalize & ESS Prep ---
          double sum_sq_weights = 0.0; 

          if (weights_sum < 1e-9) { 
             // Recovery: Reset to uniform
             const double w = inv_num_particles;
             for (auto& p : Particles) { 
                 p.weight = w; 
                 sum_sq_weights += w*w;
             }
          } else {
             const double inv = 1.0 / weights_sum;
             for (auto& p : Particles) {
                 p.weight *= inv;
                 sum_sq_weights += (p.weight * p.weight);
             }
          }

          // --- C. Calculate ESS ---
          double ESS = 1.0 / sum_sq_weights;
          
          // Print debug info periodically
          if (pros::millis() % 500 == 0) {
             printf("ESS: %.2f | ActiveSensors: %d\n", ESS, activeSensors.size());
          }

          // --- D. Resample if ESS is low ---
          if (ESS < (num_particles * 0.5)) {
              
              CDF[0] = Particles[0].weight;
              for (int i = 1; i < num_particles; ++i) {
                CDF[i] = CDF[i - 1] + Particles[i].weight;
              }

              uniform_real_distribution<double> dist(0, inv_num_particles);
              double r = dist(Random); 
              int c = 0; 
              
              for (int i = 0; i < num_particles; ++i) {
                double U = r + i * inv_num_particles;
                while (U > CDF[c] && c < num_particles - 1) {
                    c++;
                }
                Resampled[i] = Particles[c];
                Resampled[i].weight = inv_num_particles; 
              }
              Particles = Resampled;
          } 
      }
    }

    /* =============================================
       3. STATE ESTIMATION (Runs EVERY 10ms loop)
       ============================================= */
    // FIX 2: This must be OUTSIDE the 'if (doSensorUpdate)' block
    double new_x = 0.0;
    double new_y = 0.0;
    
    for (const auto& p : Particles) {
      new_x += p.x;
      new_y += p.y;
    }

    X = new_x * inv_num_particles;
    Y = new_y * inv_num_particles;

    particle_mutex.give();
    pros::Task::delay_until(&start_time, LOOP_DELAY_MS);
  }
}


  const float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI;
  const float gear_ratio = 4.0f / 3.0f;
  const float rpm_to_ips_factor = (wheel_circumference / gear_ratio) / 60.0f;

  double getAvgVelocity(void) noexcept {
    // Ensure you have access to leftMotors/rightMotors here
    const double V = (leftMotors.get_actual_velocity() * rpm_to_ips_factor + rightMotors.get_actual_velocity() * rpm_to_ips_factor) / 2.0;
    return V;
  }
}