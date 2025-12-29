#include "globals.h"

enum Color {
    NONE = 0,
    RED = 1,
    BLUE = 2
};

int get_color() {
    double hue = color_sensor.get_hue();
    Color color = NONE;
    if ((hue > 0 && hue < 50) || (hue > 310 && hue < 361)) {
        color = RED;
    }
    else if (hue > 150 && hue < 270) {
        color = BLUE;
    }
    return color;
}

void colorSort(const Color allianceColor)
{
    while(true)
    {
        Color detectedColor = static_cast<Color>(get_color());
        if (detectedColor == RED || detectedColor == BLUE) {
            if (detectedColor == allianceColor) {
                // Accept the object
                topMotor.move_velocity(200); // Move motor to accept position
            } else {
                // Reject the object
                topMotor.move_velocity(-200); // Move motor to reject position
            }
            pros::delay(200); // Wait for sorting action to complete
        }
        pros::delay(100); // Small delay to prevent excessive polling
    }
}