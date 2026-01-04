#include "velocityController.h"
#include <cmath>

int VoltageController::sign(double x) {
        return (x > 0) - (x < 0); // returns 1, 0, or -1
}

VoltageController::VoltageController(double kv, double kaStraight, double kaTurn, double ksStraight, double ksTurn, double kp,
                      double ki, double integralThreshold, double trackWidth)
        : kV(kv), kaStraight(kaStraight), kaTurn(kaTurn), ksStraight(ksStraight), ksTurn(ksTurn), kP(kp), kI(ki),
          integralThreshold(integralThreshold), trackWidth(trackWidth) {}

DrivetrainVoltages VoltageController::update(double targetLinearVelocity, double targetAngularVelocity, double measuredLeftVelocity,
                            double measuredRightVelocity) {

    // Change in target angular velocity (angular acceleration)
    double deltaW = (targetAngularVelocity - prevAngularVelocity) / 0.01;
    // Change in target linear velocity (linear acceleration)
    double deltaV = (targetLinearVelocity - prevLinearVelocity) / 0.01;

    prevAngularVelocity = targetAngularVelocity;
    prevLinearVelocity = targetLinearVelocity;

    // Differential drive kinematics
    double leftVelocity = targetLinearVelocity - targetAngularVelocity * (trackWidth / 2.0);
    double rightVelocity = targetLinearVelocity + targetAngularVelocity * (trackWidth / 2.0);

    // Velocity errors
    double leftError = leftVelocity - measuredLeftVelocity;
    double rightError = rightVelocity - measuredRightVelocity;

    // Integrals
    if ((leftError < 0) != (prevLeftError < 0)) {
        leftIntegral = 0;
    }
    if (std::abs(leftError) < integralThreshold) {
        leftIntegral += leftError * 0.01;
    }
    if ((rightError < 0) != (prevRightError < 0)) {
        rightIntegral = 0;
    }
    if (std::abs(rightError) < integralThreshold) {
        rightIntegral += rightError * 0.01;
    }

    // Feedforward Terms
    // Acceleration term when going straight (a = F/m); kaTurn from moment of inertia (α = τ/I).
    double kaLeft = (kaStraight * deltaV) - (kaTurn * deltaW);
    double kaRight = (kaStraight * deltaV) + (kaTurn * deltaW);

    // Static friction component
    double ksLeft = (ksStraight * sign(leftVelocity)) - (ksTurn * sign(targetAngularVelocity));
    double ksRight = (ksStraight * sign(rightVelocity)) + (ksTurn * sign(targetAngularVelocity));

    //double leftVoltage =
    //    std::clamp((kV * leftVelocity) + (kaLeft) + (ksLeft) + (kP * leftError) + (kI * leftIntegral), -12.0, 12.0);
    //double rightVoltage =
    //    std::clamp((kV * rightVelocity) + (kaRight) + (ksRight) + (kP * rightError) + (kI * rightIntegral), -12.0, 12.0);
    double leftVoltage =
        (kV * leftVelocity) + (kaLeft) + (ksLeft) + (kP * leftError) + (kI * leftIntegral);
    double rightVoltage =
        (kV * rightVelocity) + (kaRight) + (ksRight) + (kP * rightError) + (kI * rightIntegral);
    double ratio = std::max(fabs(leftVoltage), fabs(rightVoltage)) / 12000.0;
    if (ratio > 1) {
        leftVoltage /= ratio;
        rightVoltage /= ratio;
    }

    return {leftVoltage, rightVoltage};
}
