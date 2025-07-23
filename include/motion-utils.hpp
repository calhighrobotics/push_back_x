// motion-utils.hpp
#pragma once

#include <vector>
#include <cmath>
#include "types.hpp"

namespace MotionUtils {

    // Returns the “arc-length parameter” s for a Bezier defined by controlPts at parameter t
    float sFunction(const std::vector<Point>& controlPts, float t);

    // Returns curvature of the spline at parameter t
    float unsignedCurvature(const std::vector<Point>& controlPts, float t);
    float signedCurvature(const std::vector<Point>& controlPts, float t);

    // Returns the (x,y,θ) Pose on the spline at parameter t
    Pose findXandY(const std::vector<Point>& controlPts, float t);

    // Finds the next t such that s(t) − s0 = deltaS (e.g., by binary search or Newton)
    float findTForS(const std::vector<Point>& controlPts, float s0, float deltaS);

    // Wraps an angle into [–π, +π]
    inline float wrapAngle(float angle) {
        float wrapped = std::fmod(angle + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
        if (wrapped < 0) wrapped += 2.0f * static_cast<float>(M_PI);
        return wrapped - static_cast<float>(M_PI);
    }

    // Sinc function: sin(x)/x, with limit →1 as x→0
    inline float sinc(float x) {
        return (std::abs(x) < 1e-5f) ? 1.0f : std::sin(x) / x;
    }
}
