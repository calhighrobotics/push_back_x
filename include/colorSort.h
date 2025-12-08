#pragma once
#include "globals.h"

enum Color {
    NONE = 0,
    RED = 1,
    BLUE = 2
};

int get_color();

void colorSort(Color color);
