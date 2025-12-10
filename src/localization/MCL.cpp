#include "globals.h"
#include "cmath"
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

    vector<Particle> Particles;
    
    vector<MCLDistanceSensor> Sensors = {
      MCLDistanceSensor(frontDistance, Point(0.25, 9), FRONT),
      MCLDistanceSensor(rightDistance, Point(1.5, -5.25), RIGHT),
      MCLDistanceSensor(leftDistance, Point(1.5, 5.25), LEFT),
      MCLDistanceSensor(backDistance, Point(-3.25, -2.75), BACK), 
    };

    Field field_;
  
    vector<MCLDistanceSensor> activeSensors;

    double getAvgVelocity(void) noexcept;

    void StartMCL(double x_, double y_, double theta_);

  void MonteCarlo(void);

  const double LOOP_DELAY_MS = 10.0;
  const double LOOP_DT_SEC = LOOP_DELAY_MS / 1000.0; 

void StartMCL(double x_, double y_, double theta_) {
  Particles.clear();
  Particles.reserve(num_particles);

  uniform_real_distribution<double> start_dist_x(-start_std_pos[0], start_std_pos[0]);
  uniform_real_distribution<double> start_dist_y(-start_std_pos[1], start_std_pos[1]);
  uniform_real_distribution<double> start_dist_theta(-start_std_pos[2], start_std_pos[2]);

  for (int i = 0; i < num_particles; ++i) {
    Particles[i].x = x_ + start_dist_x(Random);
    Particles[i].y = y_ + start_dist_y(Random);
    Particles[i].theta = lemlib::radToDeg(theta_) + start_dist_theta(Random);
    Particles[i].weight = 1.0;
  }
}

double AngleWrap(double LeAngle) {
  while(LeAngle < 0) {
    LeAngle += 2.0 * M_PI;
  }
  while (LeAngle > (2 * M_PI)) {
    LeAngle -= 2.0 * M_PI;
  }
  return LeAngle;
}

void MonteCarlo(void) {
  while (true) {
    u_int32_t start_time = pros::millis();
    Velo = getAvgVelocity();

    double distance_step = Velo * LOOP_DT_SEC;
    normal_distribution<double> dist_pos(0, std::abs(distance_step * 0.2));

    const double theta_ = chassis.getPose(true).theta;
    const double rotated_theta = M_PI_2 - theta_;

    const float cos_theta = cosf(rotated_theta);
    const float sin_theta = sinf(rotated_theta);

    for (auto& p : Particles) {
      p.theta = theta_;
      p.step = Point(cos_theta, sin_theta);

      //Changed by Gemini used to be +velocity
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
          if (sensor.measurement == -1) continue;
          
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
      for (auto &p : Particles) {
        p.weight = Weight;
      }
    } else {
      const double inv = 1.0 / weights_sum;
      for (auto &p : Particles) {
        p.weight *= inv;
      }
    }

    vector<Particle> Resampled(num_particles);
    vector<double> CDF(num_particles);

    CDF[0] = Particles[0].weight;
    for (int i = 1; i < num_particles; ++i) {
      CDF[i] = CDF[i - 1] + Particles[i].weight;
    }

    uniform_real_distribution<double> dist(0, inv_num_particles);

    for (int i = 0; i < num_particles; ++i) {
      const double Point = i * (inv_num_particles) + dist(Random);
      const auto it = lower_bound(CDF.begin(), CDF.end(), Point);
      const int index = std::distance(CDF.begin(), it); 
      Resampled[i] = Particles[index];
    }

    Particles.swap(Resampled);
    
  /* Update Steps */
    double new_x = 0.0, new_y = 0.0, new_theta = 0.0, total_weight = 0.0;

    for (int i = 0; i < num_particles; ++i) {
      const auto& p = Particles[i];
      const double wt = p.weight;
      new_x += p.x * wt;
      new_y += p.y * wt;
      new_theta += p.theta * wt;
      total_weight += wt;
    }

    if (total_weight > 0) {
      const double inv_weight = 1.0 / total_weight;
      new_x *= inv_weight;
      new_y *= inv_weight;
      new_theta  *= inv_weight;
    }

    new_x = std::clamp(new_x, X - MAXSTEP, X + MAXSTEP);
    new_y = std::clamp(new_y, Y - MAXSTEP, Y + MAXSTEP);

    X = new_x, Y = new_y, theta = new_theta;

    std::cout << "X: " << X << "Y: " << Y << endl;
    lemlib::Pose current_pose = chassis.getPose();
    if(std::fabs(current_pose.x - X) < 3 && std::fabs(current_pose.y - Y) < 3)
    {
       chassis.setPose(current_pose.x * 0.6 + X * 0.4, current_pose.y * 0.6 + Y * 0.4, current_pose.theta);
    }
    else {
       chassis.setPose(current_pose.x, current_pose.y, current_pose.theta);
    }
    pros::Task::delay_until(&start_time, LOOP_DELAY_MS);
  }
}

const float wheel_circumference = 4 * M_PI;
const float gear_ratio = 1.25;
const float rpm_to_ips_factor = (wheel_circumference / gear_ratio) / 60;

double getAvgVelocity(void) noexcept {
  const double V = (leftMotors.get_actual_velocity() * rpm_to_ips_factor + rightMotors.get_actual_velocity() * rpm_to_ips_factor) / 2.0;
  return V;
}

} 