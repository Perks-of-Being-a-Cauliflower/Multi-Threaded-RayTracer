// YOU SHOULD _NOT_ NEED TO MODIFY THIS FILE

#ifndef __IMAGE_IO_H
#define __IMAGE_IO_H

// image file writing functions
//bool read_bmp(const char *name, Texture& t);
void write_bmp(const char *name, unsigned int *screen, int width, int height, int stride);
void write_tga(const char *name, unsigned int *screen, int width, int height, int stride);
void write_ppm(const char *name, unsigned int *screen, int width, int height, int stride);

#endif //__IMAGE_IO_H
