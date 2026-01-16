#include "globals.h"
#include <cmath>
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

  vector<Particle> Particles(num_particles); 

  vector<MCLDistanceSensor> Sensors = {
   MCLDistanceSensor(frontDistance, Point(-3, -0.75), FRONT),
   MCLDistanceSensor(rightDistance, Point(6.3, -0.5), RIGHT),
   MCLDistanceSensor(leftDistance, Point(-6.4, -0.5), LEFT),
   MCLDistanceSensor(backDistance, Point(-3, -10.5), BACK),
 };

  Field field_;
  vector<MCLDistanceSensor> activeSensors;
  
  pros::Mutex particle_mutex;

  double getAvgVelocity(void) noexcept;
  void StartMCL(double x_, double y_, double theta_);
  void MonteCarlo(void);

  const double LOOP_DELAY_MS = 20.0; 
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
    while (true) {
      uint32_t start_time = pros::millis();
      
      Velo = getAvgVelocity();
      double distance_step = Velo * LOOP_DT_SEC;
      normal_distribution<double> dist_pos(0, std::abs(distance_step * 0.25));

      lemlib::Pose odomPose = chassis.getPose(true);
      const double theta_ = odomPose.theta;
      const double rotated_theta = M_PI_2 - theta_;
      const float cos_theta = cosf(rotated_theta);
      const float sin_theta = sinf(rotated_theta);

      particle_mutex.take();

      for (auto& p : Particles) {
        p.theta = theta_;
        p.step = Point(cos_theta, sin_theta);
        p.x += (distance_step * cos_theta) + dist_pos(Random);
        p.y += (distance_step * sin_theta) + dist_pos(Random);
        p.x = std::clamp(p.x, -field_.HalfSize, field_.HalfSize);
        p.y = std::clamp(p.y, -field_.HalfSize, field_.HalfSize);
      }
      
      for (auto& sensor : Sensors) {
        sensor.Measure();
      }

      activeSensors.clear();
      for (auto& sensor : Sensors) {
        if (sensor.measurement > -1) {
          activeSensors.push_back(sensor);
        }
      }

      weights_sum = 0;
      if (!activeSensors.empty()) {
        for (auto& P : Particles) {
          double wt = 1.0;
          for (auto& sensor : activeSensors) {
            const double predicted = field_.get_sensor_distance(P, sensor);
            if (predicted < 0) {
              wt = 0;
              break;
            }
            Deviation = (predicted - sensor.measurement); 
            wt *= exp((Deviation * Deviation) * inv_varience) * inv_base;
          }
          weights_sum += wt;
          P.weight = wt;
        }
      }

      if (weights_sum < MIN_WEIGHT) {
        const double Weight = inv_num_particles;
        for (auto &p : Particles) p.weight = Weight;
      } else {
        const double inv = 1.0 / weights_sum;
        for (auto &p : Particles) p.weight *= inv;
      }

      CDF[0] = Particles[0].weight;
      for (int i = 1; i < num_particles; ++i) {
        CDF[i] = CDF[i - 1] + Particles[i].weight;
      }

      uniform_real_distribution<double> dist(0, inv_num_particles);
      if(Resampled.size() != num_particles) Resampled.resize(num_particles);

      for (int i = 0; i < num_particles; ++i) {
        const double sample_point = i * (inv_num_particles) + dist(Random);
        auto it = lower_bound(CDF.begin(), CDF.end(), sample_point);
        int index = std::distance(CDF.begin(), it);
        if(index >= num_particles) index = num_particles - 1; 
        Resampled[i] = Particles[index];
      }
      Particles = Resampled;

      double new_x = 0.0, new_y = 0.0, total_weight = 0.0;
      for (const auto& p : Particles) {
        new_x += p.x * p.weight;
        new_y += p.y * p.weight;
        total_weight += p.weight;
      }

      if (total_weight > 0) {
        new_x /= total_weight;
        new_y /= total_weight;
      }
      
      X = new_x;
      Y = new_y;

      particle_mutex.give();

      double dist_error = std::hypot(odomPose.x - X, odomPose.y - Y);

      cout << " time: (" << pros::millis() - start_time << ") " << std::endl;
      
      pros::Task::delay_until(&start_time, LOOP_DELAY_MS);
    }
  }

  const float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI;
  const float gear_ratio = 4.0f / 3.0f;
  const float rpm_to_ips_factor = (wheel_circumference / gear_ratio) / 60.0f;

  double getAvgVelocity(void) noexcept {
    const double V = (leftMotors.get_actual_velocity() * rpm_to_ips_factor + rightMotors.get_actual_velocity() * rpm_to_ips_factor) / 2.0;
    return V;
  }
}
