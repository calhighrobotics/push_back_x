/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

enum Color {
    NONE = 0,
    RED = 1,
    BLUE = 2
};

Color get_color();

void colorSortFn(const Color allianceColor);

void colorSort(const Color allianceColor);
