#pragma once

#include <cmath>

class Point {
  public:
    double x;
    double y;

    Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}

    // --- Vector Operations (Marked const) ---

    // Dot product
    double dot(const Point& other) const {
      return x * other.x + y * other.y;
    }

    // Squared magnitude (efficient)
    double norm_squared() const {
      return x * x + y * y;
    }

    // Magnitude
    double norm() const {
      return std::sqrt(norm_squared());
    }

    // Rotate vector
    Point rotate(float cos_theta, float sin_theta) const {
      return Point(
        x * cos_theta - y * sin_theta,
        x * sin_theta + y * cos_theta
      );
    }

    // --- Operators (Marked const) ---

    Point operator+(const Point& other) const {
      return Point(x + other.x, y + other.y);
    }

    Point operator-(const Point& other) const {
      return Point(x - other.x, y - other.y);
    }

    Point operator*(double scalar) const {
      return Point(x * scalar, y * scalar);
    }

    Point operator/(double scalar) const {
      return Point(x / scalar, y / scalar);
    }

    // Compound operators (these modify 'this', so no const)
    Point& operator+=(const Point& other) {
      x += other.x;
      y += other.y;
      return *this;
    }

    Point& operator-=(const Point& other) {
      x -= other.x;
      y -= other.y;
      return *this;
    }
};