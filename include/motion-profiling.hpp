#pragma once

#include <vector>
#include "types.hpp"
#include "motion-utils.hpp"

class TrapezoidalProfile {
public:
    TrapezoidalProfile(
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
    );

    // Advance one timestep. Returns (linear, angular, time)
    void start();
    void step();

    // True once t ≥ 1.0
    bool isFinished() const;

    // Access generated path poses & velocities
    const std::vector<Pose>& getPoses() const;
    const std::vector<VelocityLayout>& getVelocities() const;
    

private:
    // Internal state
    float s_current_;
    float prev_t_;
    float time_accum_;
    float cur_speed_;
    size_t prev_keyframe_idx_;

    // Parameters
    const std::vector<Point>& control_;
    float max_lin_vel_;
    float max_lin_accel_;
    float decel_distance_;
    float exit_velocity_;
    float initial_velocity_;       // <-- new member to store the start velocity
    bool use_keyframes_;
    float dt_;
    std::vector<KeyframeVelocities> keyframes_;

    // Accumulated output
    std::vector<Pose> poses_;
    std::vector<VelocityLayout> velocities_;

    // Helper methods
    float computeCurvatureVelocityLimit(float t) const;
    float computeAccelerationLimit(float s) const;
    float computeDecelerationLimit(float s) const;
    float computeKeyframeLimit();
    float findNextT(float s0, float deltaS) const;
};
