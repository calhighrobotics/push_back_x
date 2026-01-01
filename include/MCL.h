#pragma once

#include "main.h"
#include "lemlib/api.hpp" 
#include <cmath>
#include <random>
#include <vector>
#include <algorithm> 
#include "Point.h"

// --- Tuning Constants for Adaptive MCL ---
namespace MCLConfig {
    // Balances position vs angle differences (Paper suggests 0.8)
    static constexpr double ADAPTIVE_XI = 0.8; 
    
    // Scaling factor for density radius
    static constexpr double ADAPTIVE_ALPHA = 1.0; 

    // Minimum Noise (Inches) - Strict when converged (e.g., 2.0")
    static constexpr double SIGMA_MIN = 2.0; 

    // Maximum Noise (Inches) - Loose when lost (e.g., 20.0")
    static constexpr double SIGMA_MAX = 20.0;

    // Resampling Threshold (ESS). If ESS < N/2, we resample.
    static constexpr double ESS_RATIO = 0.5;
}

enum SensorDirection: int { FRONT = 0, LEFT = 1, BACK = 2, RIGHT = 3 };

class MCLDistanceSensor {
  public:
    MCLDistanceSensor(pros::Distance sensor_, Point Offset_, SensorDirection dir_) 
      : Sensor(sensor_), Offset(Offset_), Dir(dir_) {
      this->measurement = -1;
    }

    void Measure(void) {
      this->measurement = -1;
      // Filter out small objects/noise
      if(this->Sensor.get_object_size() < MinSize) return;

      double dist = this->Sensor.get_distance() * 0.0393701; // mm to inches
      if (dist < Range && dist > 0) {
        this->measurement = dist;
      }
    }
    
    pros::Distance Sensor;
    double measurement;
    Point Offset;
    SensorDirection Dir;
    
    static constexpr uint32_t Range = 70.866; 
    static constexpr uint32_t MinSize = 70; 
};

struct Particle {
  double x;          
  double y;          
  double theta;      
  double weight;     
  Point step;        
  double sigma;      
};

class Field {
public:
  struct Goal { 
    Point Position;
    double RadiusSquared;
    Goal (Point pos, double r) : Position(pos), RadiusSquared(r*r) {}
  };

  Field(); 

  float get_sensor_distance(const Particle& p, const MCLDistanceSensor& Sensor) const;

  static constexpr double HalfSize = 140.875 / 2.0;
  std::vector<Goal> Goals;
  
  static constexpr float direction_to_sine[] = {0, 1, 0, -1};
  static constexpr float direction_to_cosine[] = {1, 0, -1, 0};
};

namespace MCL {
  extern double X, Y, theta;
  static constexpr int num_particles = 2000;
  
  extern std::mt19937 Random;
  extern pros::Mutex particle_mutex;
  
  extern std::vector<Particle> Particles;
  extern std::vector<MCLDistanceSensor> Sensors;
  extern Field field_;

  // --- Helper: Angular Distance ---
  inline double diffAngle(double a, double b) {
      double diff = a - b;
      while (diff > M_PI) diff -= 2 * M_PI;
      while (diff < -M_PI) diff += 2 * M_PI;
      return diff;
  }

  // Function Prototypes
  extern void StartMCL(double x_ = 0, double y_ = 0, double theta_ = 0);
  extern void MonteCarlo(void);
  extern double getAvgVelocity(void);
}