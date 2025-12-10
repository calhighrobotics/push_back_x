#pragma once
#include <cmath>
#include <iostream>

using namespace std;

class Point {
  public:
    double x, y, velo, kappa;
  
  public:
  /* Constructors */
    // Default Empty Constructor
    Point() {}

    // Default Empty Destructor
    ~Point() {}

    // Default Non-Empty Constructor
    Point(double X, double Y, double C = 0, double V = 0) {
      // Setting the X,Y Values
      this->x = X, this->y = Y;
      // Setting the Other Values
      this->kappa = C, this->velo = V;
    }
  /* Getters */
    double GetX() {
      return x;
    }

    double GetY() {
      return y;
    }

    double GetVelocity() {
      return velo;
    }

    double GetCurve() {
      return kappa;
    }
  /* Setters */
    void SetX(double xVal) {
      x = xVal;
    }

    void SetY(double yVal){
      y = yVal;
    }

    void SetVelocity(double Velo) {
      velo = Velo;
    }

    void SetCurve(double Curvy) {
      kappa = Curvy;
    }

  /* Point Operations */
    Point operator+(Point P) {
      return Point(this->x + P.GetX(), this->y + P.GetY());
    }

    Point operator-(Point P) {
      return Point(this->x - P.GetX(), this->y - P.GetY());
    }

    Point operator*(Point P) {
      return Point(this->x * P.GetX(), this->y * P.GetY());
    }

    Point operator/(Point P) {
      return Point(this->x / P.GetX(), this->y / P.GetY());
    }
  /* Scalar Operations */
    Point operator+(double Scalar) {
      return Point(this->x + Scalar, this->y + Scalar);
    }

    Point operator-(double Scalar) {
      return Point(this->x - Scalar, this->y - Scalar);
    }

    Point operator*(double Scalar) {
      return Point(this->x * Scalar, this->y * Scalar);
    }

    Point operator/(double Scalar) {
      return Point(this->x / Scalar, this->y / Scalar);
    }

    Point operator^(double Scalar) {
      return Point(pow(this->x, Scalar), pow(this->y, Scalar));
    }

  /* Comparator Methods */
    bool operator==(Point &P) {
      return this->x == P.GetX() && this->y == P.GetY();
    }

    bool operator!=(Point &P) {
      return this->x != P.GetX() || this->y != P.GetY();
    }

    // Point
    Point operator+=(Point B) {
      *this = *this + B;
      return *this;
    }

    Point operator-=(Point B) {
      *this = *this - B;
      return *this;
    } 

    Point operator*=(Point B) {
      *this = *this * B;
      return *this;
    }

    Point operator/=(Point B) {
      *this = *this / B;
      return *this;
    }
    
    // Scalar
    Point operator+=(double B) {
      *this = *this + B;
      return *this;
    }

    Point operator-=(double B) {
      *this = *this - B;
      return *this;
    } 

    Point operator*=(double B) {
      *this = *this * B;
      return *this;
    }

    Point operator/=(double B) {
      *this = *this / B;
      return *this;
    }

    double Dot(Point M) {
      Point temp = *this * M;
      return temp.GetX() + temp.GetY();
    }

    double sum(void) {
      return x + y;
    }

    double operator[](double index) {
      if (index == 0)
        return x;
      return y;
    }

    double Length(void) {
      return hypotf(x, y);
    }

    double distToPoint(Point d) {
      return hypotf(this->x - d.GetX(), this->y - d.GetY());
    }

    Point rotate(float cos_theta, float sin_theta) const {
      return Point(cos_theta * this->x - sin_theta * this->y, sin_theta * this->x + cos_theta * this->y);
    }

    double norm(void) const {
      return sqrtf(this->x * this->x + this->y * this->y);
    }

    double norm_squared(void) const {
      return this->x * this->x + this->y * this->y;
    }

    double dot(const Point &v) const {
      return this->x * v.x + this->y * v.y;
    }

    double cross(const Point &v) const {
      return this->x * v.y - this->y * v.x;
    }

  ostream& operator<<(std::ostream& stream){
    return stream << "(" << this->x << ", " << this->y << ")";
  }

  /* Helper Functions */
    // Print Function
    void PrintPoint() {
      cout << "(" << this->x << ", " << this->y << ")"/*  << "\tVelo:: " << Velocity << "\tCurvature:: " << Curve */ << endl;
    }

};