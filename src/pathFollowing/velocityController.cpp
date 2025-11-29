#include <algorithm>
struct DrivetrainVoltages {
    double leftVoltage;
    double rightVoltage;
};

class VoltageController {
private:
    // Gains
    double kV;         // velocity feedforward
    double kaStraight; // acceleration feedforward
    double kaTurn;
    double ksStraight; // static friction compensation
    double ksTurn;
    double kP; // proportional gain
    double kI; // integral gain

    double trackWidth;
    double prevLinearVelocity{0.0};
    double prevAngularVelocity{0.0};
    double prevLeftError{0.0};
    double prevRightError{0.0};
    double leftIntegral{0.0};
    double rightIntegral{0.0};
    double integralThreshold{99999.0};
    double lastTargetVelocity{0.0};

    int sign(double x) {
        return (x > 0) - (x < 0); // returns 1, 0, or -1
    }

public:
    VoltageController(double kv, double kaStraight, double kaTurn, double ksStraight, double ksTurn, double kp,
                      double ki, double integralThreshold, double trackWidth)
        : kV(kv), kaStraight(kaStraight), kaTurn(kaTurn), ksStraight(ksStraight), ksTurn(ksTurn), kP(kp), kI(ki),
          integralThreshold(integralThreshold), trackWidth(trackWidth) {}

    DrivetrainVoltages update(double targetLinearVelocity, double targetAngularVelocity, double measuredLeftVelocity,
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

        double leftVoltage =
            std::clamp((kV * leftVelocity) + (kaLeft) + (ksLeft) + (kP * leftError) + (kI * leftIntegral), -12.0, 12.0);
        double rightVoltage =
            std::clamp((kV * rightVelocity) + (kaRight) + (ksRight) + (kP * rightError) + (kI * rightIntegral), -12.0, 12.0);

        return {leftVoltage, rightVoltage};
    }
};

struct VelocityControllerConfig {
    
    // --- Feedforward ---
    float kV {12.4370890785};
    float KA_turn {0.803031225567};
    float KA_straight {0.664537661342};
    float KS_turn {0.472796490892};
    float KS_straight {0.236548087393};

    // --- Feedback Gains ---
    float KP_straight {25.2621164319};
    float KI_straight {524.703492373};
};