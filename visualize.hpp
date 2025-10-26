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
const float SCALE = 150.0f; 

inline SDL_FPoint worldToScreen(float x, float y, float offsetX) {
    return {offsetX + WIDTH / 4 + x * SCALE, HEIGHT / 2 - y * SCALE};
}

inline void drawCircle(SDL_Renderer* renderer, float cx, float cy, float r) {
    for (int w = -r; w <= r; ++w) {
        for (int h = -r; h <= r; ++h) {
            if (w * w + h * h <= r * r)
                SDL_RenderDrawPointF(renderer, cx + w, cy + h);
        }
    }
}

inline void drawPoint(SDL_Renderer* renderer, const ArmConfig& cfg, float offsetX, Uint8 r, Uint8 g, Uint8 b) {
    float x = L1 * cosf(cfg.theta1) + L2 * cosf(cfg.theta1 + cfg.theta2);
    float y = L1 * sinf(cfg.theta1) + L2 * sinf(cfg.theta1 + cfg.theta2);
    SDL_FPoint p = worldToScreen(x, y, offsetX);
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);
    drawCircle(renderer, p.x, p.y, 5);
}

inline void drawArm(SDL_Renderer* renderer, const ArmConfig& cfg, float offsetX) {
    float x1 = L1 * cosf(cfg.theta1);
    float y1 = L1 * sinf(cfg.theta1);
    float x2 = x1 + L2 * cosf(cfg.theta1 + cfg.theta2);
    float y2 = y1 + L2 * sinf(cfg.theta1 + cfg.theta2);

    SDL_FPoint base = worldToScreen(0, 0, offsetX);
    SDL_FPoint joint = worldToScreen(x1, y1, offsetX);
    SDL_FPoint end = worldToScreen(x2, y2, offsetX);

    SDL_SetRenderDrawColor(renderer, 0, 128, 0, 255);
    SDL_RenderDrawLineF(renderer, base.x, base.y, joint.x, joint.y);
    SDL_RenderDrawLineF(renderer, joint.x, joint.y, end.x, end.y);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    drawCircle(renderer, base.x, base.y, 3);
    drawCircle(renderer, end.x, end.y, 3);
    drawCircle(renderer, joint.x, joint.y, 3);
}

inline void drawObstacles(SDL_Renderer* renderer, const std::vector<Obstacle>& obs, float offsetX) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    for (const auto& o : obs) {
        SDL_FPoint c = worldToScreen(o.x, o.y, offsetX);
        drawCircle(renderer, c.x, c.y, o.r * SCALE);
    }
}

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

void visualize(const std::vector<ArmConfig> &serialPath, const std::vector<ArmConfig> &parallelPath,
    const std::vector<Obstacle> &obstacles, double serialTime, double parallelTime, const std::string &testName)
{

    SDL_Window *window = SDL_CreateWindow("Arm Path Comparison", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE);
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

    Uint32 startTime = SDL_GetTicks();
    SDL_Event e;
    bool quit = false;
    int skipRate = std::max(1, (int)(serialPath.size() / 500));
    double maxTime = std::max(serialTime, parallelTime);
    double scale = 1000.0 / maxTime;
    double parallelDelay = (maxTime - serialTime) * scale;
    double serialDelay = (maxTime - parallelTime) * scale;

    size_t i = 0, j = 0;
    while (!quit) {
        while (!quit && (i < serialPath.size() || j < parallelPath.size())) {
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_QUIT) quit = true;
            }
            Uint32 elapsed = SDL_GetTicks() - startTime;

            if (elapsed > serialDelay && i < serialPath.size()) i += skipRate;
            if (elapsed > parallelDelay && j < parallelPath.size()) j += skipRate;

            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderClear(renderer);

            drawObstacles(renderer, obstacles, OFFSET_X1);
            drawObstacles(renderer, obstacles, OFFSET_X2);

            if (i < serialPath.size())  drawArm(renderer, serialPath[i], OFFSET_X1);
            else drawArm(renderer, serialPath.back(), OFFSET_X1);
            if (j < parallelPath.size()) drawArm(renderer, parallelPath[j], OFFSET_X2);
            else drawArm(renderer, parallelPath.back(), OFFSET_X2);

            drawPath(renderer, serialPath, OFFSET_X1);
            drawPath(renderer, parallelPath, OFFSET_X2);

            drawPoint(renderer, serialPath.front(), OFFSET_X1, 255, 0, 0);
            drawPoint(renderer, serialPath.back(), OFFSET_X1, 0, 255, 0);
            drawPoint(renderer, parallelPath.front(), OFFSET_X2, 255, 0, 0);
            drawPoint(renderer, parallelPath.back(), OFFSET_X2, 0, 255, 0);

            drawText("Serial", OFFSET_X1 + OFFSET_X2 / 2, 20);
            drawText("Parallel", OFFSET_X2 + OFFSET_X2 / 2, 20);
            drawText("Test: " + testName, OFFSET_X2 - 20, 10);
            SDL_SetRenderDrawColor(renderer, 150, 150, 150, 255);
            SDL_RenderDrawLine(renderer, WIDTH / 2, 50, WIDTH / 2, HEIGHT - 100);

            char stats[128];
            snprintf(stats, sizeof(stats), "Serial: %.3fs  Parallel: %.3fs  Speedup: %.2fx",
                    serialTime, parallelTime, serialTime / parallelTime);
            drawText(stats, WIDTH / 2 - 200, HEIGHT - 40);
            drawText("Time difference is not accurately represented in animation as it is spedup", WIDTH / 2 - 240, HEIGHT - 20);

            SDL_RenderPresent(renderer);
            SDL_Delay(1);
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_QUIT) { quit = true; break; }
            }
            if (quit) break;
        }

        drawText("Press r to restart", WIDTH / 2 - 80, HEIGHT / 2 + 220);
        drawText("Press SPACE to continue...", WIDTH / 2 - 100, HEIGHT / 2 + 240);
        SDL_RenderPresent(renderer);
        bool waiting = true;
        while (waiting && !quit) {
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_QUIT) { quit = true; waiting = false; break; }
                if (e.type == SDL_KEYDOWN){
                    if(e.key.keysym.sym == SDLK_SPACE) { quit = true; break; }
                    if(e.key.keysym.sym == SDLK_r){
                        waiting = false;
                        i = 0, j = 0;
                        startTime = SDL_GetTicks();
                        break;
                    }
                }
            }
            SDL_Delay(1);
        }
    }

    TTF_CloseFont(font);
    TTF_Quit();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
}