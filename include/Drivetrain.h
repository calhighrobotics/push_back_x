#pragma once

#include <utility>
#include "command/runCommand.h"
#include "command/subsystem.h"
#include "lemlib/api.hpp"
#include "pros/motor_group.hpp"


class DrivetrainSubsystem : public Subsystem {
private:
    pros::MotorGroup left11W, right11W;
    lemlib::Chassis* chassis;

public:
    DrivetrainSubsystem(const std::initializer_list<int8_t> left11_w, const std::initializer_list<int8_t> right11_w,
                        lemlib::Chassis* chassis) : left11W(left11_w), right11W(right11_w), chassis(chassis)
     {
        left11W.set_gearing_all(pros::MotorGears::blue);
        right11W.set_gearing_all(pros::MotorGears::blue);
        left11W.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        right11W.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
    }


    void periodic() override {
    }

    void setPct(const double left, const double right) {
        this->left11W.move_voltage(left * 12000.0);
        this->right11W.move_voltage(right * 12000.0);
    }

    void tank(const double left, const double right)
    {
        chassis->tank(left, right);
    }



    RunCommand *pct(double left, double right) {
        return new RunCommand([this, left, right]() { this->setPct(left, right); }, {this});
    }

    ~DrivetrainSubsystem() override = default;
};