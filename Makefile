INC_WINDOWS = include/windows
INC_LINUX   = include/linux
INC_COMMON  = include/common

# Base flags used by all platforms.
BASE_CXXFLAGS = -g -O2 -Wall -Wno-sign-compare -DHAVE_CONFIG_H

ifeq ($(OS),Windows_NT)
RM = rm -vf
INCLUDE = $(INC_WINDOWS)
CXXFLAGS = $(BASE_CXXFLAGS) -I$(INCLUDE) -I$(INC_COMMON)
CXX_EXTRA_FLAGS = -Llib -lfreeglut -lglu32 -lopengl32
 # Optional: use the exact Project 1 libpng12 path with the original 32-bit
 # course MinGW by running: make USE_LIBPNG=1
ifeq ($(USE_LIBPNG),1)
CXXFLAGS += -DIMAGEIO_USE_LIBPNG
CXX_EXTRA_FLAGS += -lpng12
endif
else
RM = rm -vf
INCLUDE = $(INC_LINUX)
CXXFLAGS = $(BASE_CXXFLAGS) -I$(INCLUDE) -I$(INC_COMMON) -I/opt/homebrew/include -DGL_SILENCE_DEPRECATION
CXX_EXTRA_FLAGS = $(shell if [ "`uname`" = "Darwin" ]; then echo "-framework GLUT -framework OpenGL -L/opt/homebrew/lib"; else echo "-lglut -lGLU -lGL"; fi)
 # Optional: use system libpng exactly like Project 1 by running:
 # make USE_LIBPNG=1
ifeq ($(USE_LIBPNG),1)
CXXFLAGS += -DIMAGEIO_USE_LIBPNG
CXX_EXTRA_FLAGS += $(shell if [ "`uname`" = "Darwin" ]; then echo "-lpng"; else echo "-lpng"; fi)
endif
endif

CXX = g++
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

# IMPORTANT:
# Do not compile every .cpp file with a wildcard here. The original Project 1
# codebase, experimental test files, or CLion scratch files may also contain an
# int main(...). A program may only have one main function, so this explicit list
# keeps ParticleToy.cpp as the single application entry point and prevents linker
# errors such as "multiple definition of main" from stale files like
# src/main.cpp or src/headless_test.cpp.
SOURCES = \
	$(SRC_DIR)/AngularSpring.cpp \
	$(SRC_DIR)/CircularWireConstraint.cpp \
	$(SRC_DIR)/CollisionHandler.cpp \
	$(SRC_DIR)/ConstraintSolver.cpp \
	$(SRC_DIR)/FluidScene.cpp \
	$(SRC_DIR)/FluidSolver2D.cpp \
	$(SRC_DIR)/GravityForce.cpp \
	$(SRC_DIR)/MouseSpringForce.cpp \
	$(SRC_DIR)/Particle.cpp \
	$(SRC_DIR)/ParticleToy.cpp \
	$(SRC_DIR)/PointConstraint.cpp \
	$(SRC_DIR)/RigidBody2D.cpp \
	$(SRC_DIR)/RodConstraint.cpp \
	$(SRC_DIR)/Solver.cpp \
	$(SRC_DIR)/SpringForce.cpp \
	$(SRC_DIR)/TouchSphereForce.cpp \
	$(SRC_DIR)/WindForce.cpp \
	$(SRC_DIR)/imageio.cpp \
	$(SRC_DIR)/linearSolver.cpp

OBJECTS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SOURCES))
EXECUTABLE = $(BIN_DIR)/project2

all: $(OBJ_DIR) $(BIN_DIR) $(EXECUTABLE)

$(OBJ_DIR):
	mkdir -p $@

$(BIN_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

$(EXECUTABLE): $(OBJECTS)
	$(CXX) -o $@ $^ $(CXX_EXTRA_FLAGS)

clean:
	@$(RM) $(OBJ_DIR)/*.o
	@$(RM) $(EXECUTABLE) $(EXECUTABLE).exe
