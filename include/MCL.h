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

    // Sensor Measurement
    void Measure(void) {
      this->measurement = -1;

      // Check if it can see the wall
      // cout << Sensor.objectRawSize() << endl;
      bool CanSeeWall = this->Sensor.get_object_size() >= MinSize;
      // CanSeeWall = true;
      
      // Get the value of the sensor
      double measurement = this->Sensor.get_distance() * 0.0393701;

      // Set the Measurement if Conditions are met
      if (CanSeeWall && measurement < Range && measurement > 0) {
        this->measurement = measurement;
      }
    }
    
    /* Variables */
    // Distance
    pros::Distance Sensor;

    // Sensor Value
    double measurement;

    // Offset from the Center Point of the Robot
    Point Offset;

    // Direction of the sensor
    SensorDirection Dir;
    
    constexpr static const uint32_t Range = 100.f; // sensor will read around 393 when it's out of range.
    constexpr static const uint32_t MinSize = 70.f; // 70
};


class Field {
public:
  struct Goal { // Mogo
    Point Position;
    double RadiusSquared;

    Goal (Point pos, double r) {
      Position = pos;
      this->RadiusSquared = r * r;
    }
  };

  // Contructor
  Field (void) {
    // Setting the Half Size

    // Vector of Mobile Goals (Representing the Poles of the Ladder)
    Goals = std::vector<Goal>({
      Goal(Point(24, 0), 2),
      Goal(Point(-24, 0), 2),
      Goal(Point(0, 24), 2),
      Goal(Point(0, -24), 2),
    }); 
    // this->Goals.reserve(4);
  }


  // Get the Length of the field
  double GetSize(void) {
    return this->HalfSize * 2.0;
  }

  // Adding a Goal
  void AddGoal(Point P, float R) {
    // Add New Goal Object
    this->Goals.push_back(Goal(P, R));
  }

  float get_sensor_distance(Particle& p, const MCLDistanceSensor& Sensor) {
    Point sensor_position = Point(p.x, p.y) + Sensor.Offset.rotate(p.step.x, p.step.y);

    // Vector
    Point step_vector = p.step.rotate(this->direction_to_cosine[Sensor.Dir], this->direction_to_sine[Sensor.Dir]);
    // Point step_vector = p.step.rotate(this->direction_to_cosine[Sensor.Dir], 1);
    // int monke = this->direction_to_sine[Sensor.Dir];
    /*  
    * Let d be the distance to the wall. Let θ be the sensor heading.
    * Take advantage of the square shape of the field to determine the distance to the wall.
    * 
    * For a vertical wall:
    * x + d cos(θ) = wall_x
    * => d = (wall_x - x) / cos(θ)
    * Intersection occurs if |y + d sin(θ)| < max_y.
    * 
    * Horizontal walls are similar.
    */
    
    // attempt the mogos first
    float min_distance = 1e10;
    bool Intersection = false;

    // constexpr float radia[] = {1, 9, 4, 9};

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
    // given the heading of the sensor, there are only two walls that can be hit
    float wall_distance;
    
    // vertical walls
    if (std::abs(step_vector.x) > 1e-4f) {
      const float wall_x = step_vector.x > 0 ? this->HalfSize : -this->HalfSize;
      wall_distance = (wall_x - sensor_position.x) / step_vector.x;
      if (std::abs(wall_distance * step_vector.y + sensor_position.y) <= this->HalfSize) {
        return wall_distance;
      }
    }
    
    // horizontal walls (it will definitely hit the horizontal walls if it didn't hit the vertical walls)
    const auto wall_y = step_vector.y > 0 ? this->HalfSize : -this->HalfSize;
    wall_distance = (wall_y - sensor_position.y) / step_vector.y;
    return wall_distance;
  }

  // Distance from Middle to Wall
  constexpr static double HalfSize = 140.875 / 2.0;

  // rotate step vector
  static constexpr float direction_to_sine[] = {0, 1, 0, -1};
  static constexpr float direction_to_cosine[] = {1, 0, -1, 0};
  
  // Vector of Goals
  std::vector<Goal> Goals;
};

namespace MCL {
  // Position Variables
  extern double X, Y, theta;

  // Total Number of Particles
  static constexpr int num_particles = 750;

  // Random Number Generator
  extern mt19937 Random;

  /* Prediction */
  extern double Velo;

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
    extern vector<Particle> Particles;

    // Vector of Distance Sensors
    extern vector<MCLDistanceSensor> Sensors;

    // The Field
    extern Field field_;

  /* Vectors */
    extern vector<MCLDistanceSensor> activeSensors;

  /* Helper Functions */
    extern double getAvgVelocity(void) noexcept;

  /* Start Function */
    extern void StartMCL(double x_ = 0, double y_ = 0, double theta_ = 0);

  /* Main Loop */
  extern void MonteCarlo(void);
}