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

    double deltaW = (targetAngularVelocity - prevAngularVelocity) / 0.01;

    double deltaV = (targetLinearVelocity - prevLinearVelocity) / 0.01;

    prevAngularVelocity = targetAngularVelocity;
    prevLinearVelocity = targetLinearVelocity;

    double leftVelocity = targetLinearVelocity - targetAngularVelocity * (trackWidth / 2.0);
    double rightVelocity = targetLinearVelocity + targetAngularVelocity * (trackWidth / 2.0);

    double leftError = leftVelocity - measuredLeftVelocity;
    double rightError = rightVelocity - measuredRightVelocity;

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

    double kaLeft = (kaStraight * deltaV) - (kaTurn * deltaW);
    double kaRight = (kaStraight * deltaV) + (kaTurn * deltaW);

    double ksLeft = (ksStraight * sign(leftVelocity)) - (ksTurn * sign(targetAngularVelocity));
    double ksRight = (ksStraight * sign(rightVelocity)) + (ksTurn * sign(targetAngularVelocity));

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
