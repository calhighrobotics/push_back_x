#pragma once
#include "types.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>

Point bezierDerivative(const std::vector<Point>& controlPoints, float t);
Point bezierSecondDerivative(const std::vector<Point>& controlPoints, float t);
float speed(const std::vector<Point>& controlPoints, float t);
float arcLength(const std::vector<Point>& controlPoints, float a, float b);
float sFunction(const std::vector<Point>& controlPoints, float t);
float findTForS(const std::vector<Point>& controlPoints, float sCurrent, float deltaS);

float bezierComponent(const std::vector<Point>& controlPoints, float t, bool useX);
bool tryNewtonRaphson(const std::vector<Point>& controlPoints, float target, bool useX, float& t);
float findTForComponent(const std::vector<Point>& controlPoints, float target, bool useX, float prevT = 0.5f);
Pose findXandY(const std::vector<Point>& controlPoints, float t);

std::vector<KeyframeVelocities> convertToTFrame(
    const std::vector<Point>& bezierPoints,
    const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocitiesXY
);

float signedCurvature(const std::vector<Point>& controlPoints, float t);
float unsignedCurvature(const std::vector<Point>& controlPoints, float t);
