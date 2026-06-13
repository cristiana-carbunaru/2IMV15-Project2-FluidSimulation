#ifndef IMAGE_IO_H
#define IMAGE_IO_H

/*
 * imageio.h

 * Project 1 exposed two helper functions for RGBA PNG files:
 *   - loadImageRGBA(...)
 *   - saveImageRGBA(...)

 * Project 2 keeps the same public interface so the rest of the application can
 * continue to request screenshots as PNG images.  The implementation has two
 * back ends:

 *   1. IMAGEIO_USE_LIBPNG defined: use the original libpng-based course code.
 *      This is the closest match to Project 1 and is useful with the original
 *      32-bit MinGW/libpng12 setup.

 *   2. IMAGEIO_USE_LIBPNG not defined: use a small built-in PNG writer/reader
 *      for CLion's 64-bit MinGW, where the old libpng12 libraries from the
 *      course bundle are binary-incompatible.  This keeps PNG screenshot output
 *      without linking to libpng12.
 */

#include <stdlib.h>

// Sets width and height and mallocs an RGBA buffer.  The caller owns the buffer
// and should release it with free().  On error, returns nullptr and sets
// width = height = -1.
unsigned char *loadImageRGBA(char *fileName, int *width, int *height);

// Saves an RGBA image to the requested file name.  Returns true on success.
bool saveImageRGBA(char *fileName, unsigned char *buffer, int width, int height);

// Returns index into image buffer for a coordinate in row-major RGBA layout.
#define indxRGBA(X,Y,W) (((Y) * (W) + (X)) * 4)

#endif
