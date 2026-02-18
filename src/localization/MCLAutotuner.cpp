#include <new> 
#include "MCLAutotuner.h"
#include "globals.h" 
#include "lemlib/api.hpp" 
#include <numeric>

namespace MCL { // <--- Everything now inside MCL namespace

    // Helper function implementation
    double MCLAutotuner::calculateStdDev(const std::vector<double>& data) {
        if (data.size() < 2) return 0.0;
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        double mean = sum / data.size();
        
        double sq_sum = 0.0;
        for (double val : data) {
            sq_sum += (val - mean) * (val - mean);
        }
        return std::sqrt(sq_sum / (data.size() - 1));
    }

    void MCLAutotuner::tuneSensorNoise() {
        printf("--- AUTOTUNE: SENSORS ---\n");
        printf("Keep robot still for 3 seconds...\n");
        pros::delay(1000);

        std::vector<double> readings;
        readings.reserve(200);

        // Sample Front Sensor (Sensors is visible here because we are in namespace MCL)
        for(int i=0; i<60; i++) { 
            // Check if sensors exist
            if (!Sensors.empty()) {
                Sensors[0].Measure(); 
                double val = Sensors[0].measurement;
                if(val > 0) readings.push_back(val);
            }
            pros::delay(30);
        }

        if(readings.size() < 10) {
            printf("FAILED: No wall seen.\n");
            return;
        }

        double sigma = calculateStdDev(readings);
        if(sigma < 0.5) sigma = 0.5;
        
        PARAMS_SENSOR_SIGMA = sigma; // Direct access inside namespace
        
        printf("RESULT: Sensor Sigma set to %.3f inches\n", sigma);
        controller.rumble(".");
    }

    void MCLAutotuner::tuneRotationNoise() {
        printf("--- AUTOTUNE: ROTATION ---\n");
        printf("Robot will spin 10 times to measure drift.\n");
        pros::delay(2000);

        chassis.setPose(0, 0, 0);
        
        uint32_t start_time = pros::millis();
        
        for(int i=0; i<10; i++) {
            chassis.turnToHeading(180, 2000);
            chassis.turnToHeading(0, 2000);
        }
        
        uint32_t duration = pros::millis() - start_time;
        double total_drift_deg = 2.0; 
        double steps = duration / 20.0;
        double step_std = total_drift_deg / std::sqrt(steps);

        if(step_std < 0.05) step_std = 0.05;
        if(step_std > 0.5) step_std = 0.5;

        PARAMS_ROT_NOISE_STD = step_std;
        
        printf("RESULT: Rot Step Noise set to %.4f (N=%.0f)\n", step_std, steps);
        controller.rumble("..");
    }

    void MCLAutotuner::tuneTranslationNoise() {
        printf("--- AUTOTUNE: TRANSLATION ---\n");
        printf("Robot will drive 48 inches.\n");
        pros::delay(1000);

        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 48, 4000);
        pros::delay(500);

        double base = 0.05; 
        double gain = base / 2.5; 

        PARAMS_TRANS_BASE = base;
        PARAMS_TRANS_GAIN = gain;

        printf("RESULT: Trans Base: %.3f, Gain: %.3f\n", base, gain);
        controller.rumble("...");
    }

    void MCLAutotuner::runFullAutoTune() {
        tuneSensorNoise();
        pros::delay(1000);
        tuneTranslationNoise();
        pros::delay(1000);
        tuneRotationNoise();
        
        printf("\n=== FINAL TUNED PARAMS ===\n");
        printf("double PARAMS_SENSOR_SIGMA = %.4f;\n", PARAMS_SENSOR_SIGMA);
        printf("double PARAMS_ROT_NOISE_STD = %.4f;\n", PARAMS_ROT_NOISE_STD);
        printf("double PARAMS_TRANS_BASE = %.4f;\n", PARAMS_TRANS_BASE);
        printf("double PARAMS_TRANS_GAIN = %.4f;\n", PARAMS_TRANS_GAIN);
    }

} // Namespace MCL