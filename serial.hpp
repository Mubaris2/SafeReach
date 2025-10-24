// serial.hpp
#ifndef SERIAL_HPP
#define SERIAL_HPP

#include "common.hpp"
#include <vector>

// Build graph and run BFS in serial. Returns sequence of ArmConfig (joint angles).
std::vector<ArmConfig> runSerial(const std::vector<Obstacle> &obstacles,
    float start_x, float start_y, float goal_x, float goal_y);

#endif
