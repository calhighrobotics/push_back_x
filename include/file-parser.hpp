// file-parser.hpp
#ifndef PATH_LOADER_HPP
#define PATH_LOADER_HPP

#include "types.hpp"
#include <string>
#include <vector>

// Parses the given file and fills controlPoints and keyFrameVelocityList
// Each time the marker is encountered, a new path block begins
void loadPaths(
    const std::string& filename,
    std::vector<std::vector<Point>>& controlPoints,
    std::vector<std::vector<KeyframeVelocitiesXandY>>& keyFrameVelocityList
);

#endif // FILE_PARSER_HPP
