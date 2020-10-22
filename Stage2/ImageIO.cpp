// YOU SHOULD _NOT_ NEED TO MODIFY THIS FILE

#include <stdio.h>
#include <iostream>
#include <fstream>
using namespace std;

#include "ImageIO.h"

void write_ppm(const char *name, unsigned int *screen, int width, int height, int stride) 
{
/*	FILE *fp = fopen(name, "w");

	fprintf(fp, "P6\n%d\n%d\n255\n", width, height);
	for (int y = 0; y < height; y++) 
	{
		for (int x = 0; x < width; x++) 
		{
			unsigned int val = screen[y * stride + x];

			fprintf(fp, "%c%c%c", val >> 16, val >> 8, val);
		}
	}
	fclose(fp);*/

}


void write_int32(ofstream& f, int value)
{
	f.put(value);
	f.put(value >> 8);
	f.put(value >> 16);
	f.put(value >> 24);
}

void write_int16(ofstream& f, int value)
{
	f.put(value);
	f.put(value >> 8);
}

void write_bmp(const char* name, unsigned int* buffer, int width, int height, int stride)
{
	ofstream imageFile(name, ios_base::binary);
	if (!imageFile) return;

	imageFile.put('B').put('M');
	write_int32(imageFile, 54 + width * height * 3);
	write_int16(imageFile, 0);
	write_int16(imageFile, 0);
	write_int32(imageFile, 54);
	write_int32(imageFile, 40);
	write_int32(imageFile, width);
	write_int32(imageFile, height);
	write_int16(imageFile, 1);
	write_int16(imageFile, 24);
	write_int32(imageFile, 0);
	write_int32(imageFile, width * height * 3);
	write_int32(imageFile, 2835);
	write_int32(imageFile, 2835);
	write_int32(imageFile, 0);
	write_int32(imageFile, 0);

	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			imageFile.put((unsigned char)(buffer[y * stride + x] >> 16)).
				put((unsigned char)(buffer[y * stride + x] >> 8)).
				put((unsigned char)(buffer[y * stride + x]));
		}
	}
}

unsigned int read_int32(ifstream& f)
{
	char value1, value2, value3, value4;
	f.get(value1);
	f.get(value2);
	f.get(value3);
	f.get(value4);

	return (((unsigned char) value4) << 24) | (((unsigned char) value3) << 16) | (((unsigned char) value2) << 8) | (unsigned char) value1;
}

unsigned int read_int16(ifstream& f)
{
	char value1, value2;
	f.get(value1);
	f.get(value2);

	return (((unsigned char) value2) << 8) | ((unsigned char) value1);
}

/*bool read_bmp(const char* name, Texture& t)
{
	ifstream imageFile(name, ios_base::binary);
	if (!imageFile) 
	{
		fprintf(stderr, "Failed to open %s.\n", name);
		return false;
	}

	char b, m;
	imageFile.get(b);
	imageFile.get(m);
	if (b != 'B' || m != 'M') 
	{
		fprintf(stderr, "File %s not a BMP.\n", name);
		return false;
	}

	int file_size = read_int32(imageFile);
	read_int32(imageFile);

	int offset = read_int32(imageFile);
	int header_size = read_int32(imageFile);

	if (offset != header_size + 14)
	{
		fprintf(stderr, "Header and offset size mismatch in %s.\n", name);
		return false;
	}

	int width = read_int32(imageFile);
	int height = read_int32(imageFile);
	int planes = read_int16(imageFile);

	int bpp = read_int16(imageFile);
	if (bpp != 24) 
	{
		fprintf(stderr, "BMP %s not 24bpp.\n", name);
		return false;
	}
	int compression = read_int32(imageFile);

	int buffer_size = read_int32(imageFile);

	int hres = read_int32(imageFile);
	int vres = read_int32(imageFile);
	int colours = read_int32(imageFile);
	int important_colours = read_int32(imageFile);

	t.width = width;
	t.height = height;
	t.sRGB = false;
	t.exposed = false;
	t.data = new unsigned int[width * height];

	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			char r, g, b;
			imageFile.get(b).get(g).get(r);
			t.data[x + y * width] = ((b & 0xFF) << 16) | ((g & 0xFF) << 8) | (r & 0xFF);
		}
		//** not handling padding at all (fails with files with a width not divisible by four) :/
	}

	return true;
}*/

void write_tga(const char* name, unsigned int* buffer, int width, int height, int stride)
{
	ofstream imageFile(name,ios_base::binary);
    if (!imageFile)
        return;
    // Addition of the TGA header
    imageFile.put(0).put(0);
    imageFile.put(2);        // RGB not compressed

    imageFile.put(0).put(0);
    imageFile.put(0).put(0);
    imageFile.put(0);

    imageFile.put(0).put(0); // origin X *
    imageFile.put(0).put(0); // origin Y 

    imageFile.put((unsigned char)(width & 0x00FF)).put((unsigned char)((width & 0xFF00) / 256));
    imageFile.put((unsigned char)(height & 0x00FF)).put((unsigned char)((height & 0xFF00) / 256));
    imageFile.put(24);                 // 24 bit bitmap
    imageFile.put(0);

	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			imageFile.put((unsigned char)(buffer[y * stride + x] >> 16)).
				put((unsigned char)(buffer[y * stride + x] >> 8)).
				put((unsigned char)(buffer[y * stride + x]));
		}
	}
}

/*
#pragma pack(2)
typedef struct tagBITMAPFILEHEADER {
  unsigned short bfType;
  unsigned int bfSize;
  unsigned short int bfReserved1;
  unsigned short int bfReserved2;
  unsigned int bfOffBits;
} BITMAPFILEHEADER;


#pragma pack()
typedef struct tagBITMAPINFOHEADER {
  unsigned int biSize;
  unsigned int biWidth;
  unsigned int biHeight;
  unsigned short biPlanes;
  unsigned short biBitCount;
  unsigned int biCompression;
  unsigned int biSizeImage;
  unsigned int biXPelsPerMeter;
  unsigned int biYPelsPerMeter;
  unsigned int biClrUsed;
  unsigned int biClrImportant;
} BITMAPINFOHEADER;
*/
