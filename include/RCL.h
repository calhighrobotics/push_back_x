/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "pros/rtos.hpp"
#include <atomic>
#include <memory>

class RCL_Controller {
private:
    std::unique_ptr<pros::Task> task;
    std::atomic<bool> running{false};

public:
    /**
     * @brief Starts the continuous distance reset task.
     * Prevents multiple tasks from spawning if already running.
     */
    void start();

    /**
     * @brief Stops the continuous distance reset task.
     */
    void end();
};

// Declare the global instance so other files know it exists
extern RCL_Controller rcl;