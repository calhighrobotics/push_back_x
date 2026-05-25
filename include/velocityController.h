/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once


struct DrivetrainVoltages {
    double leftVoltage;
    double rightVoltage;
};

class VoltageController {
private:
    double kV;
    double kaStraight;
    double kaTurn;
    double ksStraight;
    double ksTurn;
    double kP;
    double kI;

    double trackWidth;
    double prevLinearVelocity;
    double prevAngularVelocity;
    double prevLeftError;
    double prevRightError;
    double leftIntegral;
    double rightIntegral;
    double integralThreshold;
    double lastTargetVelocity;

    int sign(double x);

public:
    VoltageController(
        double kv,
        double kaStraight,
        double kaTurn,
        double ksStraight,
        double ksTurn,
        double kp,
        double ki,
        double integralThreshold,
        double trackWidth
    );

    DrivetrainVoltages update(
        double targetLinearVelocity,
        double targetAngularVelocity,
        double measuredLeftVelocity,
        double measuredRightVelocity
    );
};

struct VelocityControllerConfig {
    float kV;
    float KA_turn;
    float KA_straight;
    float KS_turn;
    float KS_straight;

    float KP_straight;
    float KI_straight;
};
