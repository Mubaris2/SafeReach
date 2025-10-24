#include <iostream>
#include <vector>
#include <omp.h>
#include "serial.hpp"
#include "parallel.hpp"
#include "visualize.hpp"
#include "common.hpp"

struct TestCase {
    std::string name;
    std::vector<Obstacle> obstacles;
    float start1, start2, goal1, goal2;
};

int main() {
    std::cout<<"Running main.cpp\n";
    std::vector<TestCase> tests = {
        {
            "1",
            {},
            -1.0f, 0.5f,
            1.0f, -0.5f
        },
        {
            "2",
            {
                {0.8f, 0.8f, 0.2f},
                {0.8f, -0.4f, 0.2f},
                {-0.8f, 0.8f, 0.2f},
                {-0.8f, -0.8f, 0.2f} 
            },
            -2.0f, 1.2f,
            2.0f, -0.2f
        },
        {
            "3",
            {
                {-0.5f, 1.2f, 0.1f},
                {-0.8f, -0.5f, 0.25f},
                {0.9f, -0.6f, 0.25f},
                {0.8f, 0.7f, 0.25f}
            },
            -2.5f, 1.4f,
            0.9f, -1.2f
        },
        {
            "4",
            {
                {-0.8f, -0.8f, 0.2f}
            },
            -2.5f, 1.4f,
            0.2f, 0.5f
        },
        {
            "5",
            {
                {0.9f, 0.6f, 0.3f},
                {0.3f, -0.2f, 0.3f}
            },
            -2.0f, 1.5f,
            3.0f, 0.5f
        }
    };

    for(auto &t:tests){
        std::cout << "Running Serial version...\n";
        double t1 = omp_get_wtime();
        auto serialPath = runSerial(t.obstacles, t.start1, t.start2, t.goal1, t.goal2);
        double t2 = omp_get_wtime();
        std::cout << "Serial completed in " << (t2 - t1) << " seconds\n";

        std::cout << "Running Parallel version...\n";
        double t3 = omp_get_wtime();
        auto parallelPath = runParallel(t.obstacles, t.start1, t.start2, t.goal1, t.goal2);
        double t4 = omp_get_wtime();
        std::cout << "Parallel completed in " << (t4 - t3) << " seconds\n";

        if (serialPath.empty() || parallelPath.empty()) {
            std::cout << "No valid path found.\n";
            continue;
        }

        // Visualize both side by side
        std::cout << "Launching SDL visualization...\n";
        visualize(serialPath, parallelPath, t.obstacles, t2 - t1, t4 - t3, t.name);
        std::cout << "Test case \"" << t.name << "\" completed.\n";
    }
    std::cout << "All test cases completed.\n";
    return 0;
}
