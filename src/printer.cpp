// printer.cpp
#include "printer.hpp"
#include <fstream>
#include <iomanip>

namespace Printer {
    // Open the output file once for all printing functions
    static std::ofstream outFile("output.txt", std::ios::out);

    void printPoseVector(
        const std::string& label,
        const std::vector<std::vector<Pose>>& poses
    ) {
        if (!outFile) {
            throw std::runtime_error("Failed to open output.txt");
        }
        outFile << label << "[";
        for (size_t i = 0; i < poses.size(); ++i) {
            for (size_t j = 0; j < poses[i].size(); ++j) {
                outFile 
                    << "("
                    << std::fixed << std::setprecision(6)
                    << poses[i][j].x << "," 
                    << poses[i][j].y 
                    << ")";
                if (i != poses.size() - 1 || j != poses[i].size() - 1) {
                    outFile << ",";
                }
            }
        }
        outFile << "]\n";
    }

    void printVelocityVector(
        const std::string& label,
        const std::vector<std::vector<VelocityLayout>>& vels,
        const std::string& whichField
    ) {
        if (!outFile) {
            throw std::runtime_error("Failed to open output.txt");
        }
        outFile << label << "[";
        for (size_t i = 0; i < vels.size(); ++i) {
            for (size_t j = 0; j < vels[i].size(); ++j) {
                float value = (whichField == "linear") 
                                ? vels[i][j].linear 
                                : vels[i][j].angular;
                outFile 
                    << "("
                    << std::fixed << std::setprecision(6)
                    << vels[i][j].time << "," 
                    << value 
                    << ")";
                if (i != vels.size() - 1 || j != vels[i].size() - 1) {
                    outFile << ",";
                }
            }
        }
        outFile << "]\n";
    }
}
