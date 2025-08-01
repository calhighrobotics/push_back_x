#pragma once
#include "lemlib/api.hpp"
#include "command/subsystem.h"
#include "command/runCommand.h"
#include "lemlib/asset.hpp"

class Drivetrain : public Subsystem {
private:

    lemlib::Chassis& chassis;

public:

    explicit Drivetrain(lemlib::Chassis& chassis)
        : chassis(chassis) { // Initialize the reference
    }
    
    /**
     * @brief Periodic runs every frame (10ms) by the command scheduler.
     */
    void periodic() override {
        std::cout << "Pose: " << chassis.getPose().x << ", " << chassis.getPose().y << ", " << chassis.getPose().theta << std::endl;
    }


    void Tank(const double left, const double right) {
        this->chassis.tank(left, right);
    }

    void moveTo(double x, double y, double timeout) {
        chassis.moveToPoint(x, y, timeout);
    }

    void turnTo(double heading, double timeout)
    {
        chassis.turnToHeading(heading, timeout);
    }

    void follow_path(auto path, int timeout)
    {
        chassis.follow(path, 15, timeout);
    }

    RunCommand* driverControl(const std::function<double()>& left_stick, const std::function<double()>& right_stick) {
        return new RunCommand(
            [this, left_stick, right_stick] () {
                this->Tank(left_stick(), right_stick());
            },
            {this}
        );
    }

    RunCommand* moveToPoint(const double x, const double y, const double timeout) {
        return new RunCommand(
            [this, x, y, timeout] () {
                this->moveTo(x, y, timeout);
            },
            {this}
        );
    }

    RunCommand* turnToPoint(const double heading, const double timeout)
    {
        return new RunCommand(
            [this, heading, timeout] ()
            {
                this->turnTo(heading, timeout);
            },
            {this}
        );
    }

    RunCommand* followPath(auto path, const int timeout)
    {
        return new RunCommand(
            [this, path, timeout] ()
            {
                this->follow_path(path, timeout);
            },
            {this}
        );
    }
    


    ~Drivetrain() override = default;
};
