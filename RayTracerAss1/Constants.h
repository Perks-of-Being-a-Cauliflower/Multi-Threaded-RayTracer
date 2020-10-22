/*  The following code is a VERY heavily modified from code originally sourced from:
	Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
	It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

#ifndef __CONSTANTS_H
#define __CONSTANTS_H

#include <algorithm>

// maximum size of image
const int MAX_WIDTH = 2048, MAX_HEIGHT = 2048;

// math constants
const float PI = 3.14159265358979323846f;
const float PIOVER180 = 0.017453292519943295769236907684886f;

// a small value (used to make sure we don't get stuck detecting collision of the same object over and over) 
const float EPSILON = 0.01f;

// maximum ray distance
const float MAX_RAY_DISTANCE = 2000000.0f;  //** maybe should be maxfloat?!

// the maximum number of rays to cast before giving up on finding final ray destination
const int MAX_RAYS_CAST = 10; 

// default refractive index (of air effectively)
const float DEFAULT_REFRACTIVE_INDEX = 1.0f;

#endif //__CONSTANTS_H
