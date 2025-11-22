#pragma once
#include <vector>
#include <utility>
#include "types.hpp"

class AStar {
public:
    AStar(const std::vector<std::vector<char>> &map);
    std::pair<int,int> computeNextMove(int start_x, int start_y, int goal_x, int goal_y);

private:
    const std::vector<std::vector<char>> &map_;
};
