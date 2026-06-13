#pragma once

// Cross-platform GLUT include used by both Project 1 and Project 2 code.
// macOS ships GLUT as a framework; Linux/Windows commonly expose GL/glut.h.
// This way we won't have to change the framework everytime based on our operating system.
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
