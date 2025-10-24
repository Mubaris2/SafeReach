#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <SDL_ttf.h>
#include <vector>
#include <cmath>
#include "common.hpp"

const int WIDTH = 1200;
const int HEIGHT = 600;
const float OFFSET_X1 = 0.0f;
const float OFFSET_X2 = WIDTH / 2.0f;
const float OFFSET_Y = HEIGHT / 2.0f;
const float SCALE = 150.0f;  // pixels per unit length

// Convert world coordinates to screen coordinates
inline SDL_FPoint worldToScreen(float x, float y, float offsetX) {
    return {offsetX + WIDTH / 4 + x * SCALE, HEIGHT / 2 - y * SCALE};
}

// Draw a filled circle
inline void drawCircle(SDL_Renderer* renderer, float cx, float cy, float r) {
    for (int w = -r; w <= r; ++w) {
        for (int h = -r; h <= r; ++h) {
            if (w * w + h * h <= r * r)
                SDL_RenderDrawPointF(renderer, cx + w, cy + h);
        }
    }
}

// Draw a point representing start/goal
inline void drawPoint(SDL_Renderer* renderer, const ArmConfig& cfg, float offsetX, Uint8 r, Uint8 g, Uint8 b) {
    float x = L1 * cosf(cfg.theta1) + L2 * cosf(cfg.theta1 + cfg.theta2);
    float y = L1 * sinf(cfg.theta1) + L2 * sinf(cfg.theta1 + cfg.theta2);
    SDL_FPoint p = worldToScreen(x, y, offsetX);
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);
    drawCircle(renderer, p.x, p.y, 5);
}

// Draw arm configuration
inline void drawArm(SDL_Renderer* renderer, const ArmConfig& cfg, float offsetX) {
    float x1 = L1 * cosf(cfg.theta1);
    float y1 = L1 * sinf(cfg.theta1);
    float x2 = x1 + L2 * cosf(cfg.theta1 + cfg.theta2);
    float y2 = y1 + L2 * sinf(cfg.theta1 + cfg.theta2);

    SDL_FPoint base = worldToScreen(0, 0, offsetX);
    SDL_FPoint joint = worldToScreen(x1, y1, offsetX);
    SDL_FPoint end = worldToScreen(x2, y2, offsetX);

    // Arm segments (green)
    SDL_SetRenderDrawColor(renderer, 0, 128, 0, 255);
    SDL_RenderDrawLineF(renderer, base.x, base.y, joint.x, joint.y);
    SDL_RenderDrawLineF(renderer, joint.x, joint.y, end.x, end.y);

    // Arm joint (black)
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    drawCircle(renderer, base.x, base.y, 3);
    drawCircle(renderer, end.x, end.y, 3);
    drawCircle(renderer, joint.x, joint.y, 3);
}

// Draw obstacles
inline void drawObstacles(SDL_Renderer* renderer, const std::vector<Obstacle>& obs, float offsetX) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    for (const auto& o : obs) {
        SDL_FPoint c = worldToScreen(o.x, o.y, offsetX);
        drawCircle(renderer, c.x, c.y, o.r * SCALE);
    }
}

// Draw the entire path (as yellow trace)
inline void drawPath(SDL_Renderer* renderer, const std::vector<ArmConfig>& path, float offsetX) {
    if (path.empty()) return;
    SDL_SetRenderDrawColor(renderer, 255, 215, 0, 255);
    for (size_t i = 1; i < path.size(); ++i) {
        float x1 = L1 * cosf(path[i - 1].theta1) + L2 * cosf(path[i - 1].theta1 + path[i - 1].theta2);
        float y1 = L1 * sinf(path[i - 1].theta1) + L2 * sinf(path[i - 1].theta1 + path[i - 1].theta2);
        float x2 = L1 * cosf(path[i].theta1) + L2 * cosf(path[i].theta1 + path[i].theta2);
        float y2 = L1 * sinf(path[i].theta1) + L2 * sinf(path[i].theta1 + path[i].theta2);
        SDL_FPoint p1 = worldToScreen(x1, y1, offsetX);
        SDL_FPoint p2 = worldToScreen(x2, y2, offsetX);
        SDL_RenderDrawLineF(renderer, p1.x, p1.y, p2.x, p2.y);
    }
}

void visualize(
    const std::vector<ArmConfig> &serialPath,
    const std::vector<ArmConfig> &parallelPath,
    const std::vector<Obstacle> &obstacles,
    double serialTime, double parallelTime, const std::string &testName)
{

    SDL_Window *window = SDL_CreateWindow("Arm Path Comparison",
                                          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          WIDTH, HEIGHT,
                                          SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    TTF_Init();
    TTF_Font *font = TTF_OpenFont("C:/Windows/Fonts/arial.ttf", 18);

    auto drawText = [&](const std::string &text, int x, int y) {
        SDL_Color color = {0, 0, 0, 255};
        SDL_Surface *surface = TTF_RenderText_Blended(font, text.c_str(), color);
        if (!surface) return;
        SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (texture) {
            SDL_Rect dst = {x, y, surface->w, surface->h};
            SDL_RenderCopy(renderer, texture, nullptr, &dst);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    };    

    // double maxTime = std::max(serialTime, parallelTime);
    // double delay1 = maxTime / std::max<size_t>(1, serialPath.size());
    // double delay2 = maxTime / std::max<size_t>(1, parallelPath.size());
    
    bool quit = false;
    SDL_Event e;
    while (!quit) {
        size_t maxFrames = std::max(serialPath.size(), parallelPath.size());
        for (size_t i = 0; i < maxFrames && !quit; ++i) {
            // Poll events at start of frame and exit immediately if requested
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_QUIT) { quit = true; break; }
            }
            if (quit) break;
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderClear(renderer);
            
            drawObstacles(renderer, obstacles, OFFSET_X1);
            drawObstacles(renderer, obstacles, OFFSET_X2);

            // Mark start and goal positions
            drawPoint(renderer, serialPath.front(), OFFSET_X1, 255, 0, 0); // Start (red)
            drawPoint(renderer, serialPath.back(), OFFSET_X1, 0, 255, 0);  // Goal (green)
            drawPoint(renderer, parallelPath.front(), OFFSET_X2, 255, 0, 0); // Start (red)
            drawPoint(renderer, parallelPath.back(), OFFSET_X2, 0, 255, 0);  // Goal (green)
            
            // Draw arms
            if (i < serialPath.size()) drawArm(renderer, serialPath[i], OFFSET_X1);
            if (i < parallelPath.size()) drawArm(renderer, parallelPath[i], OFFSET_X2);
            
            drawPath(renderer, serialPath, OFFSET_X1);
            drawPath(renderer, parallelPath, OFFSET_X2);

            // Labels
            drawText("Serial", OFFSET_X1 + OFFSET_X2 / 2, 30);
            drawText("Parallel", OFFSET_X2 + OFFSET_X2 / 2, 30);
            drawText("Test: " + testName, OFFSET_X2 - 20, 10);

            // Divider line
            SDL_SetRenderDrawColor(renderer, 150, 150, 150, 255);
            SDL_RenderDrawLine(renderer, WIDTH / 2, 50, WIDTH / 2, HEIGHT - 100);

            char stats[256];
            snprintf(stats, sizeof(stats),
                     "Serial: %.3fs   Parallel: %.3fs   Speedup: %.2fx",
                     serialTime, parallelTime, serialTime / parallelTime);
            drawText(stats, WIDTH / 2 - 200, HEIGHT - 40);

            SDL_RenderPresent(renderer);
            SDL_Delay(100);
            // Poll again after delay to handle a close during sleep
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_QUIT) { quit = true; break; }
            }
            if (quit) break;
        }

        // Finished – wait for SPACE to continue
        drawText("Press SPACE to continue...", WIDTH / 2 - 100, HEIGHT / 2 + 200);
        SDL_RenderPresent(renderer);
        // Wait for SPACE or quit. This loop exits immediately if SDL_QUIT is received.
        bool waiting = true;
        while (waiting && !quit) {
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_QUIT) { quit = true; waiting = false; break; }
                if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) { waiting = false; break; }
            }
            SDL_Delay(10);
        }
    }

    TTF_CloseFont(font);
    TTF_Quit();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
}
