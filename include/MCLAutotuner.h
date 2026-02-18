#pragma once
#include "main.h"
#include "MCL.h"
#include <vector>
#include <cmath>
#include <numeric>

namespace MCL { // <--- Added Namespace

    class MCLAutotuner {
    public:
        /**
         * @brief Phase 1: Calibrate Distance Sensor Noise (Sigma)
         * Robot must be stationary, facing a wall approx 24" away.
         */
        void tuneSensorNoise();

        /**
         * @brief Phase 2: Calibrate Rotation Noise
         * Robot will spin 360 degrees (10 times) and measure Odom drift.
         */
        void tuneRotationNoise();

        /**
         * @brief Phase 3: Calibrate Translation Noise
         * Robot will drive forward 48" and back 48".
         */
        void tuneTranslationNoise();

        /**
         * @brief Runs all routines sequentially.
         */
        void runFullAutoTune();

    private:
        // Helper to calculate standard deviation of a vector
        double calculateStdDev(const std::vector<double>& data);
    };

} // <--- End Namespace