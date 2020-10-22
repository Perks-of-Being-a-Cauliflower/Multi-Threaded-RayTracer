/*  The following code is a VERY heavily modified from code originally sourced from:
	Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
	It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

#ifndef __COLOUR_H
#define __COLOUR_H

#include <algorithm>
#include <math.h>

// a colour consists of three primary components (red, green, and blue)
struct Colour 
{
    float red, green, blue;

	// create uninitialised colour
	inline Colour() { }

	// create initialised colour
	inline Colour(float r, float g, float b) : red(r), green(g), blue(b)
	{
	}

	// create colour from integer (pixel stored in 0x00BBGGRR format)
	inline Colour(unsigned int value) :
		red(float(value & 0xFF) / 255.0f),
		green(float((value >> 8) & 0xFF) / 255.0f),
		blue(float((value >> 16) & 0xFF) / 255.0f)
	{
	}

	// convert colour to pixel (in 0x00BBGGRR format) with respect to an exposure level 
	inline unsigned int convertToPixel(float exposure)
	{
		return ((unsigned char) (255 * (std::min(1.0f - expf(blue * exposure), 1.0f))) << 16) +
			((unsigned char) (255 * (std::min(1.0f - expf(green * exposure), 1.0f))) << 8) +
			((unsigned char) (255 * (std::min(1.0f - expf(red * exposure), 1.0f))) << 0);
	}

	// convert colour to pixel (in 0x00BBGGRR format)
	inline unsigned int convertToPixel()
	{
		return ((unsigned char)(255 * (std::min(blue, 1.0f))) << 16) +
			((unsigned char)(255 * (std::min(green, 1.0f))) << 8) +
			((unsigned char)(255 * (std::min(red, 1.0f))) << 0);
	}

	// colour += colour
	inline Colour& operator += (const Colour& c2) 
	{
	    this->red +=  c2.red;
        this->green += c2.green;
        this->blue += c2.blue;
	    return *this;
    }
};

// colour * colour
inline Colour operator * (const Colour& c1, const Colour& c2) 
{
	return Colour(c1.red * c2.red, c1.green * c2.green, c1.blue * c2.blue);
}

// colour + colour
inline Colour operator + (const Colour& c1, const Colour& c2) 
{
	return Colour(c1.red + c2.red, c1.green + c2.green, c1.blue + c2.blue);
}

// float * colour (float multiplied to each colour channel)
inline Colour operator * (float coef, const Colour& c1) 
{
	return Colour(c1.red * coef, c1.green * coef, c1.blue * coef);
}

#endif //__COLOUR_H
