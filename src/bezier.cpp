#include "bezier.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>

// Compute derivative of a cubic Bezier curve
Point bezierDerivative(const std::vector<Point>& controlPoints, float t) {
	float dx = 3 * (1 - t) * (1 - t) * (controlPoints[1].x - controlPoints[0].x) +
		6 * (1 - t) * t * (controlPoints[2].x - controlPoints[1].x) +
		3 * t * t * (controlPoints[3].x - controlPoints[2].x);
	float dy = 3 * (1 - t) * (1 - t) * (controlPoints[1].y - controlPoints[0].y) +
		6 * (1 - t) * t * (controlPoints[2].y - controlPoints[1].y) +
		3 * t * t * (controlPoints[3].y - controlPoints[2].y);
	return {dx, dy};
}
// Compute second derivative of a cubic Bezier curve
Point bezierSecondDerivative(const std::vector<Point>& controlPoints, float t) {
	float dx = 6 * (1 - t) * (controlPoints[2].x - 2 * controlPoints[1].x + controlPoints[0].x) +
		6 * t * (controlPoints[3].x - 2 * controlPoints[2].x + controlPoints[1].x);
	float dy = 6 * (1 - t) * (controlPoints[2].y - 2 * controlPoints[1].y + controlPoints[0].y) +
		6 * t * (controlPoints[3].y - 2 * controlPoints[2].y + controlPoints[1].y);
	return {dx, dy};
}
// Compute speed function ||r'(t)||
float speed(const std::vector<Point>& controlPoints, float t) {
	Point deriv = bezierDerivative(controlPoints, t);
	return std::sqrt(deriv.x * deriv.x + deriv.y * deriv.y);
}
// Compute arc length using Gaussian quadrature
float arcLength(const std::vector<Point>& controlPoints, float a, float b) {
	// Gaussian quadrature weights and nodes for n=5, if using quadratic bezier n=3
	const std::vector<float> gaussNodes = {-0.9061798459, -0.5384693101, 0.0, 0.5384693101, 0.9061798459};
	const std::vector<float> gaussWeights = {0.2369268850, 0.4786286705, 0.5688888889, 0.4786286705, 0.2369268850};
	float length = 0.0;
	for (size_t i = 0; i < gaussNodes.size(); i++) {
		float t = (b - a) / 2.0 * gaussNodes[i] + (a + b) / 2.0;
		length += gaussWeights[i] * speed(controlPoints, t);
	}
	return (b - a) / 2.0 * length;
}
// Compute s(t) from 0 to t
float sFunction(const std::vector<Point>& controlPoints, float t) {
	return arcLength(controlPoints, 0, t);
}
// Newton-Raphson to find t for s(t) = s_current + delta_s
float findTForS(const std::vector<Point>& controlPoints, float sCurrent, float deltaS) {
	float tol = 1e-6;
	float t = 0.5; // Initial guess
	int maxIter = 20;
	for (int i = 0; i < maxIter; i++) {
		float s_t = sFunction(controlPoints, t);
		float f_t = s_t - sCurrent - deltaS;
		float f_prime_t = speed(controlPoints, t);

		if (std::fabs(f_t) < tol) break;
		t -= f_t / f_prime_t;
		if (t < 0) t = 0;
		if (t > 1) t = 1;
	}
	return t;
}

// Evaluate cubic Bézier component at t
float bezierComponent(const std::vector<Point>& controlPoints, float t, bool useX) {
    float u = 1 - t;
    float c0 = useX ? controlPoints[0].x : controlPoints[0].y;
    float c1 = useX ? controlPoints[1].x : controlPoints[1].y;
    float c2 = useX ? controlPoints[2].x : controlPoints[2].y;
    float c3 = useX ? controlPoints[3].x : controlPoints[3].y;

    return u*u*u * c0 +
           3*u*u*t * c1 +
           3*u*t*t * c2 +
           t*t*t * c3;
}

// Try Newton-Raphson starting from tGuess
bool tryNewtonRaphson(const std::vector<Point>& controlPoints, float target, bool useX, float& t) {
    float tol = 1e-6f;
    int maxIter = 20;

    for (int i = 0; i < maxIter; ++i) {
        float val = bezierComponent(controlPoints, t, useX);
        Point deriv = bezierDerivative(controlPoints, t);
        float d = useX ? deriv.x : deriv.y;
        float f = val - target;

        if (std::fabs(f) < tol) return true;
        if (std::fabs(d) < 1e-10f) return false;

        t -= f / d;
        if (t < 0.0f || t > 1.0f) return false; // Stay in bounds
    }
    return true;
}

// Main function: find t for x(t) = xTarget or y(t) = yTarget
float findTForComponent(
    const std::vector<Point>& controlPoints,
    float target,          // xTarget or yTarget
    bool useX,             // true for x(t), false for y(t)
    float prevT     // for disambiguating multiple solutions
) {
    std::vector<float> candidates;

    // Try Newton-Raphson from several seeds
    for (float seed : {0.2f, 0.5f, 0.8f}) {
        float t = seed;
        if (tryNewtonRaphson(controlPoints, target, useX, t)) {
            // Avoid duplicates within tolerance
            bool exists = false;
            for (float existing : candidates) {
                if (std::fabs(existing - t) < 1e-4f) {
                    exists = true;
                    break;
                }
            }
            if (!exists) candidates.push_back(t);
        }
    }

    // Fallback: sample curve if Newton-Raphson fails
    if (candidates.empty()) {
        int samples = 100;
        float bestT = 0.0f;
        float minDiff = 1e10f;
        for (int i = 0; i <= samples; ++i) {
            float t = i / float(samples);
            float val = bezierComponent(controlPoints, t, useX);
            float diff = std::fabs(val - target);
            if (diff < minDiff) {
                minDiff = diff;
                bestT = t;
            }
        }
        candidates.push_back(bestT);
    }

    // Return the t closest to prevT
    auto closest = std::min_element(candidates.begin(), candidates.end(),
        [prevT](float a, float b) {
            return std::fabs(a - prevT) < std::fabs(b - prevT);
        });
    
    return *closest;
}

std::vector<KeyframeVelocities> convertToTFrame(
	const std::vector<Point>& bezierPoints,
	const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocitiesXY
) {
	std::vector<KeyframeVelocities> keyFrameVelocitiesT;
	float prevT = 0.0f;

	for (const auto& kf : keyFrameVelocitiesXY) {
        float t = findTForComponent(bezierPoints, kf.x, true, prevT);

		keyFrameVelocitiesT.push_back({kf.velocity, t});
		prevT = t; // update for next iteration
	}
	return keyFrameVelocitiesT;
}
// Compute signed curvature Îº(t)
float signedCurvature(const std::vector<Point>& controlPoints, float t) {
	Point r1 = bezierDerivative(controlPoints, t);
	Point r2 = bezierSecondDerivative(controlPoints, t);

	// Compute cross product magnitude for 2D case (determinant form)
	float crossProduct = (r1.x * r2.y - r1.y * r2.x);

	// Compute denominator |r'(t)|^3
	float speedCubed = std::pow(std::sqrt(r1.x * r1.x + r1.y * r1.y), 3);

	// Avoid division by zero
	if (speedCubed < 1e-6) return 0.0;

	return crossProduct / speedCubed;
}

// Compute unsigned curvature Îº(t)
float unsignedCurvature(const std::vector<Point>& controlPoints, float t) {
	Point r1 = bezierDerivative(controlPoints, t);
	Point r2 = bezierSecondDerivative(controlPoints, t);

	// Compute cross product magnitude for 2D case (determinant form)
	float crossProduct = (r1.x * r2.y - r1.y * r2.x);

	// Compute denominator |r'(t)|^3
	float speedCubed = std::pow(std::sqrt(r1.x * r1.x + r1.y * r1.y), 3);

	// Avoid division by zero
	if (speedCubed < 1e-6) return 0.0;

	return fabs(crossProduct) / speedCubed;
}

Pose findXandY(const std::vector<Point>& controlPoints, float t) {
	float x = std::pow(1 - t, 3) * controlPoints[0].x +
		3 * std::pow(1 - t, 2) * t * controlPoints[1].x +
		3 * (1 - t) * std::pow(t, 2) * controlPoints[2].x +
		std::pow(t, 3) * controlPoints[3].x;

	float y = std::pow(1 - t, 3) * controlPoints[0].y +
		3 * std::pow(1 - t, 2) * t * controlPoints[1].y +
		3 * (1 - t) * std::pow(t, 2) * controlPoints[2].y +
		std::pow(t, 3) * controlPoints[3].y;
	Point derivative = bezierDerivative(controlPoints, t);
	float theta = std::atan2(derivative.y, derivative.x);
	return {x, y, theta};
}
