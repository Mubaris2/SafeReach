// serial.cpp
#include "serial.hpp"
#include <queue>
#include <iostream>
#include <algorithm>

std::vector<ArmConfig> runSerial(const std::vector<Obstacle> &obstacles,
                                float start_x, float start_y,
                                float goal_x, float goal_y)
{
    int W = theta2Count(); // columns (t2 index)
    int H = theta1Count(); // rows (t1 index)
    int N = W * H;

    // 1) Precompute valid nodes (collision-free)
    std::vector<bool> valid(N, false);
    for (int i = 0; i < H; ++i) {
        float t1 = indexToTheta1(i);
        for (int j = 0; j < W; ++j) {
            float t2 = indexToTheta2(j);
            int idx = idxFromIJ(i,j,W);
            valid[idx] = !checkCollision(t1, t2, obstacles);
        }
    }

    // 2) Find nearest valid nodes to start and goal end-effector positions
    int startIdx = findNearestValidNodeToXY(start_x, start_y, valid, W, H);
    int goalIdx  = findNearestValidNodeToXY(goal_x,  goal_y,  valid, W, H);

    if (startIdx < 0 || goalIdx < 0) {
        std::cerr << "Serial: no valid start/goal node found.\n";
        return {};
    }

    // 3) BFS on grid graph (4-neighborhood in (i,j) indices)
    std::vector<int> parent(N, -1);
    std::vector<char> visited(N, 0);
    std::queue<int> q;
    visited[startIdx] = 1;
    parent[startIdx] = -1;
    q.push(startIdx);

    int found = 0;
    while (!q.empty()) {
        int cur = q.front(); q.pop();
        if (cur == goalIdx) { found = 1; break; }
        int ci, cj; ijFromIdx(cur, W, ci, cj);
        const int di[4] = {1,-1,0,0}; // only single-joint moves
        const int dj[4] = {0,0,1,-1};
        for (int k=0;k<4;++k) {
            int ni = ci + di[k], nj = cj + dj[k];
            if (ni < 0 || ni >= H || nj < 0 || nj >= W) continue;
            int nidx = idxFromIJ(ni, nj, W);
            if (!valid[nidx]) continue;
            if (visited[nidx]) continue;
            visited[nidx] = 1;
            parent[nidx] = cur;
            q.push(nidx);
        }
    }

    if (!found) {
        std::cerr << "Serial: no path found between snapped start/goal.\n";
        return {};
    }

    // 4) reconstruct path
    std::vector<ArmConfig> path;
    int cur = goalIdx;
    while (cur != -1) {
        int ci, cj; ijFromIdx(cur, W, ci, cj);
        path.push_back({ indexToTheta1(ci), indexToTheta2(cj) });
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}
