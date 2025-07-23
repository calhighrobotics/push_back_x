#include "motion-profiling.hpp"
#include <algorithm>
#include <cmath>

using namespace MotionUtils;

TrapezoidalProfile::TrapezoidalProfile(
    const std::vector<Point>& controlPts,
    float maxLinVel,
    float maxLinAccel,
    float decelDist,
    float timeAccum,
    float startVel,
    float endVel,
    const std::vector<KeyframeVelocities>& keyframes,
    bool useKeyframes,
    float dt
)
    : s_current_(-100.0f),
      prev_t_(0.0f),
      time_accum_(timeAccum),
      cur_speed_(startVel),
      prev_keyframe_idx_(0),
      control_(controlPts),
      max_lin_vel_(maxLinVel),
      max_lin_accel_(maxLinAccel),
      decel_distance_(decelDist),
      exit_velocity_(endVel),
      initial_velocity_(startVel),   // store the start velocity
      use_keyframes_(useKeyframes),
      dt_(dt),
      keyframes_(keyframes)
{
    poses_.reserve(1000);
    velocities_.reserve(1000);
}

bool TrapezoidalProfile::isFinished() const {
    return prev_t_ >= 1.0f;
}

const std::vector<Pose>& TrapezoidalProfile::getPoses() const {
    return poses_;
}

const std::vector<VelocityLayout>& TrapezoidalProfile::getVelocities() const {
    return velocities_;
}

float TrapezoidalProfile::computeCurvatureVelocityLimit(float t) const {
    const float trackWidth = 0.288925f;
    float curv = unsignedCurvature(control_, t);
    if (std::abs(curv) < 1e-6f) {
        return max_lin_vel_;
    }
    float turn_radius = 1.0f / curv;
    return max_lin_vel_ * turn_radius / (turn_radius + trackWidth / 2.0f);
}

float TrapezoidalProfile::computeAccelerationLimit(float s) const {
    // If still in early “acceleration” phase (before decelerationDistance), ramp up:
    if (s < decel_distance_) {
        return cur_speed_ + (max_lin_accel_ * dt_);
    }

    // Otherwise, compute the “coast / decel” region:
    float total_length = sFunction(control_, 1.0f);    
    float accel_dist = (pow(max_lin_vel_, 2) - pow(initial_velocity_, 2)) / (2 * max_lin_accel_);
    if (s > accel_dist) {
        float linear_velocity_decel_limit = sqrt(pow(exit_velocity_, 2) - ((total_length - s) * (pow(exit_velocity_, 2) - pow(max_lin_vel_, 2)) / (total_length - decel_distance_)));
        return (linear_velocity_decel_limit);
    }

    // If we haven't reached “acc_dist” yet, just return current speed
    return cur_speed_;
}

float TrapezoidalProfile::computeDecelerationLimit(float) const {
    return cur_speed_ - (max_lin_accel_ * dt_);
}

float TrapezoidalProfile::computeKeyframeLimit() {
    if (!use_keyframes_ || prev_keyframe_idx_ + 1 >= keyframes_.size()) {
        return std::numeric_limits<float>::infinity();
    }

    const auto& kf0 = keyframes_[prev_keyframe_idx_];
    const auto& kf1 = keyframes_[prev_keyframe_idx_ + 1];

    if (kf1.time < time_accum_) {
        ++prev_keyframe_idx_;
    }

    float vi2 = kf0.velocity * kf0.velocity;
    float vf2 = kf1.velocity * kf1.velocity;
    float s_now = sFunction(control_, prev_t_);
    float s_kf1 = sFunction(control_, kf1.time);

    if (s_kf1 <= 0.0f) {
        return kf1.velocity;
    }
    float ratio = s_now / s_kf1;
    float vel_lim_sq = vi2 + (vf2 - vi2) * ratio;
    return (vel_lim_sq > 0.0f ? std::sqrt(vel_lim_sq) : kf1.velocity);
}

float TrapezoidalProfile::findNextT(float s0, float deltaS) const {
    return findTForS(control_, s0, deltaS);
}
void TrapezoidalProfile::start() {
    Pose newPose = findXandY(control_, 0.0f);
    poses_.push_back(newPose);
    VelocityLayout vlay{ 0.0f, 0.0f, time_accum_ };
    velocities_.push_back(vlay);
}

void TrapezoidalProfile::step() {
    time_accum_ += dt_;
    s_current_ = sFunction(control_, prev_t_);

    float keyframe_lim  = computeKeyframeLimit();
    float curvature_lim = computeCurvatureVelocityLimit(prev_t_);
    float accel_lim     = computeAccelerationLimit(s_current_);
    float decel_lim     = computeDecelerationLimit(s_current_);

    float desired_linear = std::min({ curvature_lim,
                                      accel_lim,
                                      keyframe_lim,
                                      max_lin_vel_ });
    // desired_linear = std::max({desired_linear, decel_lim});
    float deltaS = desired_linear * dt_;
    float next_t = findNextT(s_current_, deltaS);

    float kappa = signedCurvature(control_, next_t);
    float turning_component = kappa * desired_linear;

    Pose newPose = findXandY(control_, next_t);
    poses_.push_back(newPose);

    VelocityLayout vlay{ desired_linear, turning_component, time_accum_ };
    velocities_.push_back(vlay);

    prev_t_    = next_t;
    cur_speed_ = desired_linear;           
}
