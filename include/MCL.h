#pragma once

#include "globals.h"
#include <cmath>
#include <random>
#include "Point.h"
#include <vector>
#include <map>


template <typename T>
T clamp(const T& val, const T& lowerBound, const T& upperBound) {
  if (val < lowerBound) {
    return lowerBound;
  } else if (val > upperBound) {
    return upperBound;
  } else {
    return val;
  }
}

struct Particle {
  double x;          // X position
  double y;          // Y position
  double theta;      // Orientation

  // Probability Weight
  double weight;

  // Change in Position (ΔX, ΔY)
  Point step;
};

enum SensorDirection: int {
  FRONT = 0,
  LEFT = 1,
  BACK = 2,
  RIGHT = 3,
};

class MCLDistanceSensor {
  public:
    MCLDistanceSensor(pros::Distance sensor_,Point Offset_, SensorDirection dir_) : Sensor(sensor_), Offset(Offset_), Dir(dir_) {
      this->measurement = -1;
    }
    void Measure(void) {
      this->measurement = -1;

      bool CanSeeWall = this->Sensor.get_object_size() >= MinSize;
      
      double measurement = this->Sensor.get_distance() * 0.0393701;

      if (CanSeeWall && measurement < Range && measurement > 0) {
        this->measurement = measurement;
      }
    }
    
    pros::Distance Sensor;


    double measurement;

    Point Offset;

    SensorDirection Dir;
    
    constexpr static const uint32_t Range = 100.f; 
    constexpr static const uint32_t MinSize = 70.f; 
};


class Field {
public:
  struct Goal { 
    Point Position;
    double RadiusSquared;

    Goal (Point pos, double r) {
      Position = pos;
      this->RadiusSquared = r * r;
    }
  };

  Field (void) {
    Goals = std::vector<Goal>({
      //Long Goals
      Goal(Point(-24, -48 + 2.5), 2.5),
      Goal(Point(24, -48 + 2.5), 2.5),
      Goal(Point(-24, 48 - 2.5), 2.5),
      Goal(Point(24, 48 - 2.5), 2.5),
      //MatchLoaders
      Goal(Point(-72 + 2.5,-48), 2.3),
      Goal(Point(-72 + 2.5,48), 2.3),
      Goal(Point(72 - 2.5,-48), 2.3),
      Goal(Point(72 - 2.5,48), 2.3),
      // Midgoal
      /*
      Goal(Point(-3.41,3.41), 2.75),
      Goal(Point(3.41,-3.41), 2.75),
      Goal(Point(0,0), 2.75),
      */
      Goal(Point(0,0), 4),
    }); 
  }


  double GetSize(void) {
    return this->HalfSize * 2.0;
  }

  void AddGoal(Point P, float R) {
    this->Goals.push_back(Goal(P, R));
  }

  float get_sensor_distance(Particle& p, const MCLDistanceSensor& Sensor) {
    Point sensor_position = Point(p.x, p.y) + Sensor.Offset.rotate(p.step.x, p.step.y);

    Point step_vector = p.step.rotate(this->direction_to_cosine[Sensor.Dir], this->direction_to_sine[Sensor.Dir]);
    
    float min_distance = 1e10;
    bool Intersection = false;

    for (Goal &G : this->Goals) {
      Point v = G.Position - sensor_position;
      if (((step_vector.x < 0) == (v.x < 0) && (step_vector.y < 0) == (v.y < 0)) || v.norm_squared() <= G.RadiusSquared) {
        double proj = v.dot(step_vector);
        double perp_squared = v.norm_squared() - proj * proj;

        if (perp_squared <= G.RadiusSquared) {
          auto t = proj - std::sqrt(G.RadiusSquared - perp_squared);
          if (t <= min_distance) {
            Intersection = true;
            min_distance = t;
          }   
        }
      }
    }

    if (Intersection) {
      return min_distance;
    }
    float wall_distance;
    
    if (std::abs(step_vector.x) > 1e-4f) {
      const float wall_x = step_vector.x > 0 ? this->HalfSize : -this->HalfSize;
      wall_distance = (wall_x - sensor_position.x) / step_vector.x;
      if (std::abs(wall_distance * step_vector.y + sensor_position.y) <= this->HalfSize) {
        return wall_distance;
      }
    }
    const auto wall_y = step_vector.y > 0 ? this->HalfSize : -this->HalfSize;
    wall_distance = (wall_y - sensor_position.y) / step_vector.y;
    return wall_distance;
  }

  constexpr static double HalfSize = 140.875 / 2.0;

  static constexpr float direction_to_sine[] = {0, 1, 0, -1};
  static constexpr float direction_to_cosine[] = {1, 0, -1, 0};
  
  std::vector<Goal> Goals;
};

namespace MCL {
  extern double X, Y, theta;

  static constexpr int num_particles = 2000;

  extern std::mt19937 Random;

  /* Prediction */
  extern double Velo;
  extern pros::Mutex particle_mutex;
  /* Update */

    // Double
    extern double Deviation;
    extern double weights_sum;
    extern double start_theta;

    // Static Constants
    static constexpr double inv_num_particles = 1.0 / num_particles;
    static constexpr double MAXSTEP = .67;
    static constexpr double toRad = M_PI / 180.0;
    static constexpr double sqrt_2_pi = 2.506628275;
    static constexpr double VeloScale = .1275 * .004;
    static constexpr double std_sensor = 2.0; // Standard deviation of Distance Sensor (Inches)
    static constexpr double MIN_WEIGHT = 1e-30;
    static constexpr double inv_varience = -.5 / (std_sensor * std_sensor);
    static constexpr double inv_base = 1 / (std_sensor * sqrt_2_pi);

  /* Arrays */
    // Standard Deviations
    static constexpr double std_pos [] = {.5, .5, 0};
    // Starting
    static constexpr double start_std_pos [] = {6, 6, M_PI / 360};
  /* Vectors */
    // Vectors of Particles
    extern std::vector<Particle> Particles;

    // Vector of Distance Sensors
    extern std::vector<MCLDistanceSensor> Sensors;

    // The Field
    extern Field field_;

  /* Vectors */
    extern std::vector<MCLDistanceSensor> activeSensors;

  /* Helper Functions */
    extern double getAvgVelocity(void) noexcept;

  /* Start Function */
    extern void StartMCL(double x_ = 0, double y_ = 0, double theta_ = 0);

  /* Main Loop */
  extern void MonteCarlo(void);
}