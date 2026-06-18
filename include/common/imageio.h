#ifndef IMAGE_IO_H
#define IMAGE_IO_H

#include <stdlib.h>

unsigned char *loadImageRGBA(char *fileName, int *width, int *height);

// Saves an RGBA image to the requested file name.  Returns true on success.
bool saveImageRGBA(char *fileName, unsigned char *buffer, int width, int height);

// Returns index into image buffer for a coordinate in row-major RGBA layout.
#define indxRGBA(X,Y,W) (((Y) * (W) + (X)) * 4)

#endif
