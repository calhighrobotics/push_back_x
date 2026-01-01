#include "globals.h"
#include "pros/rtos.hpp"

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

void colorSortFn(const Color allianceColor)
{
    while(true)
    {
        Color detectedColor = static_cast<Color>(get_color());
        if (detectedColor == RED || detectedColor == BLUE) {
            if (detectedColor == allianceColor) {
                topMotor.move_velocity(200);
            } else {
                topMotor.move_velocity(-200); 
            }
            pros::delay(200); 
        }
        pros::delay(100);
    }
}

void colorSort(const Color allianceColor)
{
    pros::Task colorSortTask([allianceColor]() {
        colorSortFn(allianceColor);
    });
    
}