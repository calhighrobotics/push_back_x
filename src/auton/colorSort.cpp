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
    if ((hue > 40 && hue < 60)) {
        color = RED;
    }
    else if (hue > 100 && hue < 150) {
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
                //topMotor.move_velocity(200);
            } else {
                topMotor.move_voltage(-8 * 1000);
            }
            pros::delay(200); 
            topMotor.brake();
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