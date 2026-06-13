cmake_minimum_required(VERSION 3.16)
project(Project2IMV15_FluidSimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (APPLE)
    set(PLATFORM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/linux)
elseif (WIN32)
    set(PLATFORM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/windows)
else()
    set(PLATFORM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/linux)
endif()

# Keep this list explicit for the same reason as the Makefile: only
# ParticleToy.cpp contains the application main(...). Do not glob src/*.cpp,
# because any helper/test file with its own main would break the link step.
set(PROJECT_SOURCES
    src/AngularSpring.cpp
    src/CircularWireConstraint.cpp
    src/CollisionHandler.cpp
    src/ConstraintSolver.cpp
    src/FluidScene.cpp
    src/FluidSolver2D.cpp
    src/GravityForce.cpp
    src/MouseSpringForce.cpp
    src/Particle.cpp
    src/ParticleToy.cpp
    src/PointConstraint.cpp
    src/RigidBody2D.cpp
    src/RodConstraint.cpp
    src/Solver.cpp
    src/SpringForce.cpp
    src/TouchSphereForce.cpp
    src/WindForce.cpp
    src/imageio.cpp
    src/linearSolver.cpp
)

add_executable(project2 ${PROJECT_SOURCES})

target_include_directories(project2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common
    ${PLATFORM_INCLUDE_DIR}
)

target_compile_definitions(project2 PRIVATE HAVE_CONFIG_H GL_SILENCE_DEPRECATION)

if (APPLE)
    find_library(GLUT_FRAMEWORK GLUT REQUIRED)
    find_library(OPENGL_FRAMEWORK OpenGL REQUIRED)
    target_link_libraries(project2 PRIVATE ${GLUT_FRAMEWORK} ${OPENGL_FRAMEWORK})
elseif (WIN32)
    # CLion's bundled MinGW is normally 64-bit. The original course libraries
    # in lib/ are 32-bit, which produced linker messages such as
    # "skipping incompatible libfreeglut.a" and "skipping incompatible
    # libpng12.a". For a 64-bit compiler, use the supplied x64 freeglut library
    # and let imageio.cpp use its built-in PNG writer instead of libpng12.
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        target_compile_definitions(project2 PRIVATE FREEGLUT_STATIC)
        target_link_libraries(project2 PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/x64/libfreeglut_static.a
            glu32 opengl32 winmm gdi32
        )
    else()
        # Legacy 32-bit course/MinGW setup.  This matches Project 1: use the
        # original libpng12-backed imageio implementation.
        target_compile_definitions(project2 PRIVATE IMAGEIO_USE_LIBPNG)
        target_link_directories(project2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)
        target_link_libraries(project2 PRIVATE freeglut glu32 opengl32 png12)
    endif()
else()
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    target_include_directories(project2 PRIVATE ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
    target_link_libraries(project2 PRIVATE OpenGL::GL GLUT::GLUT)

    find_package(PNG QUIET)
    if (PNG_FOUND)
        # On Linux/macOS, use the system libpng when it is available.  If it is
        # not installed, imageio.cpp automatically falls back to its built-in PNG
        # screenshot writer.
        target_compile_definitions(project2 PRIVATE IMAGEIO_USE_LIBPNG)
        target_include_directories(project2 PRIVATE ${PNG_INCLUDE_DIRS})
        target_link_libraries(project2 PRIVATE PNG::PNG)
    endif()
endif()

set_target_properties(project2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
