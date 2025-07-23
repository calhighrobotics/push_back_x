#include "file-parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

void loadPaths(
    const std::string& filename,
    std::vector<std::vector<Point>>& controlPoints,
    std::vector<std::vector<KeyframeVelocitiesXandY>>& keyFrameVelocityList
) {
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Error: could not open " << filename << "\n";
        return;
    }

    enum class State { None, ReadingPoints, ReadingVels };
    State state = State::None;

    std::string line;
    std::vector<Point>    currentPoints;
    std::vector<KeyframeVelocitiesXandY> currentVels;

    auto pushBlock = [&]() {
        if (!currentPoints.empty()) {
            controlPoints.push_back(currentPoints);
            currentPoints.clear();
            // if no velocities were specified, push one default
            if (currentVels.empty()) {
                keyFrameVelocityList.emplace_back(1, KeyframeVelocitiesXandY{0.f,0.f,0.f});
            } else {
                keyFrameVelocityList.push_back(currentVels);
            }
            currentVels.clear();
        }
    };

    while (std::getline(infile, line)) {
        // trim leading whitespace
        auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        line = line.substr(first);

        if (line.rfind("#PATH-START", 0) == 0) {
            // if we were mid‑block, push it
            if (state == State::ReadingVels) {
                pushBlock();
            }
            state = State::None;
            continue;
        }
        if (line.rfind("#PATH.JERRYIO-DATA", 0) == 0) {
            // if we were mid‑block, push it
            if (state == State::ReadingVels) {
                pushBlock();
            }
            state = State::None;
            continue;
        }
        if (line.rfind("#POINTS-START", 0) == 0) {
            // if we just finished a velocities block, push it before starting a new one
            if (state == State::ReadingVels) {
                pushBlock();
            }
            currentPoints.clear();
            state = State::ReadingPoints;
            continue;
        }
        if (line.rfind("#VELOCITIES-START", 0) == 0) {
            state = State::ReadingVels;
            currentVels.clear();
            continue;
        }

        std::istringstream iss(line);
        if (state == State::ReadingPoints) {
            float x, y;
            char comma;
            if (iss >> x >> comma >> y && comma == ',') {
                currentPoints.push_back(Point{x, y});
            } else {
                std::cerr << "Warning: failed to parse point line: " << line << "\n";
            }
        }
        else if (state == State::ReadingVels) {
            float vx, vy, vz;
            char c1, c2;
            if (iss >> vx >> c1 >> vy >> c2 >> vz && c1 == ',' && c2 == ',') {
                currentVels.push_back(KeyframeVelocitiesXandY{vx, vy, vz});
            } else {
                std::cerr << "Warning: failed to parse velocity line: " << line << "\n";
            }
        }
    }

    // at EOF, if we were mid‑block, push it
    if (state == State::ReadingVels) {
        pushBlock();
    }
}
