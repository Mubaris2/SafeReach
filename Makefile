# Compiler and flags
CXX := g++
CXXFLAGS := -std=c++20 -O2 -Wall -fopenmp

# SDL2 include & lib paths (adjust only if your setup differs)
SDL_CFLAGS := -IC:/msys64/ucrt64/include/SDL2
SDL_LDLIBS := -LC:/msys64/ucrt64/lib

# SDL2 libraries
LIBS := -lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf

# Source and object files
SRC := main.cpp serial.cpp parallel.cpp
OBJ := $(SRC:.cpp=.o)
TARGET := compare.exe

# Default target
all: $(TARGET)

# Linking object files to create the executable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(SDL_LDLIBS) $(LIBS)

# Compiling each .cpp file into .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(SDL_CFLAGS) -c $< -o $@

# Run the program
run: $(TARGET)
	./$(TARGET)

# Clean object files and executable
clean:
	rm -f $(OBJ) $(TARGET)
