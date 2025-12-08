#include "velocityController.h"
#include <algorithm>

// Constructor
VoltageController::VoltageController(
    double kv,
    double kaStraight,
    double kaTurn,
    double ksStraight,
    double ksTurn,
    double kp,
    double ki,
    double integralThreshold,
    double trackWidth
)
    : kV(kv),
      kaStraight(kaStraight),
      kaTurn(kaTurn),
      ksStraight(ksStraight),
      ksTurn(ksTurn),
      kP(kp),
      kI(ki),
      integralThreshold(integralThreshold),
      trackWidth(trackWidth),
      prevLinearVelocity(0.0),
      prevAngularVelocity(0.0),
      prevLeftError(0.0),
      prevRightError(0.0),
      leftIntegral(0.0),
      rightIntegral(0.0),
      lastTargetVelocity(0.0)
{}

// sign helper
int VoltageController::sign(double x) {
    return (x > 0) - (x < 0);
}

DrivetrainVoltages VoltageController::update(
    double targetLinearVelocity,
    double targetAngularVelocity,
    double measuredLeftVelocity,
    double measuredRightVelocity
) {
    double deltaW = (targetAngularVelocity - prevAngularVelocity) / 0.01;
    double deltaV = (targetLinearVelocity - prevLinearVelocity) / 0.01;

    prevAngularVelocity = targetAngularVelocity;
    prevLinearVelocity = targetLinearVelocity;

    // Kinematics
    double leftVelocity  = targetLinearVelocity - targetAngularVelocity * (trackWidth / 2.0);
    double rightVelocity = targetLinearVelocity + targetAngularVelocity * (trackWidth / 2.0);

    // Errors
    double leftError  = leftVelocity  - measuredLeftVelocity;
    double rightError = rightVelocity - measuredRightVelocity;

    // Integral reset on sign flip
    if ((leftError < 0) != (prevLeftError < 0))
        leftIntegral = 0;

    if (std::abs(leftError) < integralThreshold)
        leftIntegral += leftError * 0.01;

    if ((rightError < 0) != (prevRightError < 0))
        rightIntegral = 0;

    if (std::abs(rightError) < integralThreshold)
        rightIntegral += rightError * 0.01;

    prevLeftError  = leftError;
    prevRightError = rightError;

    // Feedforward
    double kaLeft  = (kaStraight * deltaV) - (kaTurn * deltaW);
    double kaRight = (kaStraight * deltaV) + (kaTurn * deltaW);

    double ksLeft  = (ksStraight * sign(leftVelocity))  - (ksTurn * sign(targetAngularVelocity));
    double ksRight = (ksStraight * sign(rightVelocity)) + (ksTurn * sign(targetAngularVelocity));

    double leftVoltage =
        std::clamp(kV * leftVelocity  + kaLeft  + ksLeft  + kP * leftError  + kI * leftIntegral, -12.0, 12.0);

    double rightVoltage =
        std::clamp(kV * rightVelocity + kaRight + ksRight + kP * rightError + kI * rightIntegral, -12.0, 12.0);

    return {leftVoltage, rightVoltage};
}
