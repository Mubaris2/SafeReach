#ifndef PARALLEL_HPP
#define PARALLEL_HPP

#include "common.hpp"
#include <vector>

std::vector<ArmConfig> runParallel(const std::vector<Obstacle> &obstacles,float start_x, float start_y, float goal_x, float goal_y);

#endif
