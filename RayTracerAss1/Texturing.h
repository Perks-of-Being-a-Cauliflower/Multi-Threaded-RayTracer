#ifndef __TEXTURING_H
#define __TEXTURING_H

#include "Scene.h"
#include "Intersection.h"

// apply computed checkerboard texture
Colour applyCheckerboard(const Intersection* intersect);

// apply computed turbulence texture
Colour applyCircles(const Intersection* intersect);

// apply computed wood texture
Colour applyWood(const Intersection* intersect);

#endif // __TEXTURING_H
