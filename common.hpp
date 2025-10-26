#ifndef COMMON_HPP
#define COMMON_HPP

#include <vector>
#include <cmath>
#include <limits>

constexpr float L1 = 0.7f;
constexpr float L2 = 1.2f;
constexpr float THETA1_STEP = 0.003f;
constexpr float THETA2_STEP = 0.003f;
constexpr float WAYPOINT_TOLERANCE = 0.18f; 
constexpr float M_PI = 3.14159265358979323846f;

struct Obstacle { float x, y, r; };
struct ArmConfig { float theta1, theta2; };

inline void forwardKinematics(float t1, float t2, float &x, float &y) {
    x = L1 * std::cos(t1) + L2 * std::cos(t1 + t2);
    y = L1 * std::sin(t1) + L2 * std::sin(t1 + t2);
}

inline bool lineCircleCollision(float x1, float y1, float x2, float y2, const Obstacle &obs) {
    float dx = x2 - x1, dy = y2 - y1;
    float fx = x1 - obs.x, fy = y1 - obs.y;
    float a = dx*dx + dy*dy;
    float b = 2*(fx*dx + fy*dy);
    float c = fx*fx + fy*fy - obs.r*obs.r;
    float disc = b*b - 4*a*c;
    if (disc < 0) return false;
    disc = std::sqrt(disc);
    float t1 = (-b - disc)/(2*a);
    float t2 = (-b + disc)/(2*a);
    return (t1 >= 0.0f && t1 <= 1.0f) || (t2 >= 0.0f && t2 <= 1.0f);
}

inline bool checkCollision(float t1, float t2, const std::vector<Obstacle> &obstacles) {
    float x0 = 0.0f, y0 = 0.0f;
    float x1 = L1 * std::cos(t1), y1 = L1 * std::sin(t1);
    float x2 = x1 + L2 * std::cos(t1 + t2), y2 = y1 + L2 * std::sin(t1 + t2);
    for (const auto &obs : obstacles) {
        if (lineCircleCollision(x0, y0, x1, y1, obs)) return true;
        if (lineCircleCollision(x1, y1, x2, y2, obs)) return true;
    }
    return false;
}

inline int theta1Count() {
    return int(std::floor((2.0f * M_PI) / THETA1_STEP)) + 1;
}
inline int theta2Count() {
    return int(std::floor((2.0f * M_PI) / THETA2_STEP)) + 1;
}
inline float indexToTheta1(int i) {
    return -M_PI + i * THETA1_STEP;
}
inline float indexToTheta2(int j) {
    return -M_PI + j * THETA2_STEP;
}
inline int idxFromIJ(int i, int j, int W) { return i * W + j; }
inline void ijFromIdx(int idx, int W, int &i, int &j) { i = idx / W; j = idx % W; }

inline int findNearestValidNodeToXY(float xgoal, float ygoal, const std::vector<bool> &valid, int W, int H) {
    int bestIdx = -1;
    float bestDist2 = std::numeric_limits<float>::infinity();
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            int idx = idxFromIJ(i,j,W);
            if (!valid[idx]) continue;
            float t1 = indexToTheta1(i);
            float t2 = indexToTheta2(j);
            float x,y; forwardKinematics(t1,t2,x,y);
            float d2 = (x - xgoal)*(x - xgoal) + (y - ygoal)*(y - ygoal);
            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestIdx = idx;
            }
        }
    }
    return bestIdx;
}

#endif 