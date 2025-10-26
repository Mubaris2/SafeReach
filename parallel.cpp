#include "parallel.hpp"
#include <queue>
#include <iostream>
#include <omp.h>
#include <algorithm>

std::vector<ArmConfig> runParallel(const std::vector<Obstacle> &obstacles, float start_x, float start_y, float goal_x, float goal_y) {
    int W = theta2Count();
    int H = theta1Count();
    int N = W * H;

    std::vector<bool> valid(N, false);
    int validCount = 0;
    #pragma omp parallel for reduction(+:validCount) schedule(dynamic, 32)
    for (int idx = 0; idx < N; ++idx) {
        int i = idx / W;
        int j = idx % W;
        float t1 = indexToTheta1(i);
        float t2 = indexToTheta2(j);
        bool coll = checkCollision(t1, t2, obstacles);
        if (!coll) {
            valid[idx] = true;
            validCount += 1;
        } else {
            valid[idx] = false;
        }
    }

    if (validCount == 0) {
        std::cerr << "Parallel: no valid configs at all.\n";
        return {};
    }

    auto nearest_search = [&](float gx, float gy) {
        int bestIdx = -1;
        float bestDist2 = std::numeric_limits<float>::infinity();
        #pragma omp parallel
        {
            int localBest = -1;
            float localBestDist2 = std::numeric_limits<float>::infinity();
            #pragma omp for nowait
            for (int idx = 0; idx < N; ++idx) {
                if (!valid[idx]) continue;
                int i = idx / W;
                int j = idx % W;
                float t1 = indexToTheta1(i);
                float t2 = indexToTheta2(j);
                float x,y; forwardKinematics(t1,t2,x,y);
                float d2 = (x - gx)*(x - gx) + (y - gy)*(y - gy);
                if (d2 < localBestDist2) {
                    localBestDist2 = d2;
                    localBest = idx;
                }
            }
            #pragma omp critical
            {
                if (localBest >= 0 && localBestDist2 < bestDist2) {
                    bestDist2 = localBestDist2;
                    bestIdx = localBest;
                }
            }
        } 
    int startIdx = nearest_search(start_x, start_y);
    int goalIdx  = nearest_search(goal_x,  goal_y);

    if (startIdx < 0 || goalIdx < 0) {
        std::cerr << "Parallel: no valid start/goal node found.\n";
        return {};
    }

    std::vector<int> parent(N, -1);
    std::vector<char> visited(N, 0);
    std::queue<int> q;
    visited[startIdx] = 1;
    q.push(startIdx);
    bool found = false;
    while (!q.empty() && !found) {
        int cur = q.front(); q.pop();
        if (cur == goalIdx) { found = true; break; }
        int ci = cur / W, cj = cur % W;
        const int di[4] = {1,-1,0,0};
        const int dj[4] = {0,0,1,-1};
        for (int k=0;k<4;++k) {
            int ni = ci + di[k], nj = cj + dj[k];
            if (ni < 0 || ni >= H || nj < 0 || nj >= W) continue;
            int nidx = idxFromIJ(ni,nj,W);
            if (!valid[nidx]) continue;
            if (visited[nidx]) continue;
            visited[nidx] = 1;
            parent[nidx] = cur;
            q.push(nidx);
        }
    }

    if (!found) {
        std::cerr << "Parallel: no path found between snapped start/goal.\n";
        return {};
    }

    std::vector<ArmConfig> path;
    int cur = goalIdx;
    while (cur != -1) {
        int ci = cur / W, cj = cur % W;
        path.push_back({ indexToTheta1(ci), indexToTheta2(cj) });
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}