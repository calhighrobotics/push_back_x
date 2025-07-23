// motion_utils.cpp
#include "motion-utils.hpp"
#include "bezier.hpp"

namespace MotionUtils {

    float sFunction(const std::vector<Point>& controlPts, float t) {
        return ::sFunction(controlPts, t);
    }

    float unsignedCurvature(const std::vector<Point>& controlPts, float t) {
        return ::unsignedCurvature(controlPts, t);
    }
    float signedCurvature(const std::vector<Point>& controlPts, float t) {
        return ::signedCurvature(controlPts, t);
    }

    Pose findXandY(const std::vector<Point>& controlPts, float t) {
        return ::findXandY(controlPts, t);
    }

    float findTForS(const std::vector<Point>& controlPts, float s0, float deltaS) {
        return ::findTForS(controlPts, s0, deltaS);
    }

    // wrapAngle and sinc are inline in the header
}
