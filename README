# 🦾 SafeReach

A C++ project that visualizes motion planning for a 2-link robotic arm using BFS and OpenMP.
It compares serial vs parallel exploration of configuration space and visualizes both executions side by side using SDL2.

## 🛠 Dependencies

Make sure these following SDL2 libraries are installed:

- SDL2 >= 2.0.22
- SDL2_ttf >= 2.20.2 (for text rendering)

### On MSYS2 (Windows)

```bash
pacman -S mingw-w64-x86_64-SDL2 mingw-w64-x86_64-SDL2_ttf
```

### On Ubuntu/Debian (Linux)

```bash
sudo apt install libsdl2-dev libsdl2-ttf-dev
```

## 🏗️ Build Instructions

This project uses a `Makefile` for easy compilation.

### 1. Clone the repo

```bash
git clone https://github.com/Mubaris2/SafeReach.git
cd SafeReach
```

### 2. Edit `Makefile`

Set your SDL2 include and lib paths:

```make
# Example Makefile
SDL2_CFLAGS := -IC:/msys64/mingw64/include/SDL2
SDL2_LIBS := -LC:/msys64/mingw64/lib
```

### 3. Compile

```bash
make run
```

## 🎮 How to Use

- 5 Testcases are hardcoded and will run back to back.
- After running a test case, there are 2 options;
  - Press `r` to rewatch the same testcase.
  - Press `Space` to watch the next testcase.
- Serial time, Parallel time and SpeedUp will be available underneath the animation.

## 📁 Project Structure

```
SafeReach
├── main.cpp
├── serial.cpp
├── parallel.cpp
├── common.hpp
├── serial.hpp
├── parallel.hpp
├── Makefile
└── README.md
```