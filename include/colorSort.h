#pragma once

enum Color {
    NONE = 0,
    RED = 1,
    BLUE = 2
};

Color get_color();

void colorSortFn(const Color allianceColor);

void colorSort(const Color allianceColor);
