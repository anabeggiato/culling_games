#pragma once
#include <cmath>
#include <utility>

// Estrutura de nó do A*
struct AStarNode {
    int x, y;
    int g, h, f;
    AStarNode(int X = 0, int Y = 0, int G = 0, int H = 0)
        : x(X), y(Y), g(G), h(H), f(G + H) {}
};

// Comparador para priority_queue
struct CompareAStar {
    bool operator()(const AStarNode &a, const AStarNode &b) const {
        if (a.f != b.f) return a.f > b.f;
        return a.h > b.h;
    }
};

// Heurística Manhattan
inline int manhattan(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}
