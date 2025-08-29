// motionUtilities_zya.hpp
#pragma once
#include <cmath>

#define MAX_SPEED 1.0
#define EPSILON 0.1
#define DRONE_RADIUS 0.5  // 0.8/2 + 0.1安全余量

struct Coordinate {
    double x, y, z;
};

inline bool equalCoord(const Coordinate& a, const Coordinate& b) {
    return std::hypot(a.x-b.x, a.y-b.y) < EPSILON && std::abs(a.z-b.z) < EPSILON;
}