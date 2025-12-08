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

void colorSort(const Color color)
{
    
}