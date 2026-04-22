#include "RCL.h"
#include "distanceReset.h"
#include "globals.h"

RCL_Controller rcl;

void RCL_Controller::start() {
    if (running.exchange(true)) {
    return; 
    }

    task = std::make_unique<pros::Task>([this]() {
    const double alpha = 0.1; 
    
    while (running) {
        distancePose targetXY = distanceReset(false, true, 2, 10); 
        
        lemlib::Pose currentPose = chassis.getPose(); 
        
        float newX = currentPose.x + alpha * (targetXY.x - currentPose.x);
        float newY = currentPose.y + alpha * (targetXY.y - currentPose.y);
        
        chassis.setPose({newX, newY, currentPose.theta}); 
        pros::delay(50); 
    }
    }, "RCL Task");
}

void RCL_Controller::end() {
    if (running.exchange(false)) {
        pros::delay(60); 
    }
}