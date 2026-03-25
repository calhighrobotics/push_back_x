#include "globals.h"
#include "pros/rtos.hpp"


Color get_color() {
    double hue = color_sensor.get_hue();
    Color color = NONE;
    if ((hue > 10 && hue < 30)) {
        color = RED;
    }
    else if (hue > 190 && hue < 250) {
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
