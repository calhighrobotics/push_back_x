#include "RCL.h"
#include "distanceReset.h"

RCL_Controller rcl;

void RCL_Controller::start() {
    if (running.exchange(true)) {
        return; 
    }
    task = std::make_unique<pros::Task>([this]() {
        while (running) {
            distanceReset(true, true, 3.5, 10);
            pros::delay(200); 
        }
    }, "RCL Task");
}

void RCL_Controller::end() {
    if (running.exchange(false)) {
        pros::delay(60); 
    }
}