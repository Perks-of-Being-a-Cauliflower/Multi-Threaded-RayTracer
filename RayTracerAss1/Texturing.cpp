#include "Texturing.h"
#include "Colour.h"
#include "Intersection.h"

// apply computed checkerboard texture
Colour applyCheckerboard(const Intersection* intersect)
{
	Point p = (intersect->pos - intersect->material->offset) / intersect->material->size;

	int which = (int(floorf(p.x)) + int(floorf(p.y)) + int(floorf(p.z))) & 1;

	return (which ? intersect->material->diffuse : intersect->material->diffuse2);
}


// apply computed circular texture
Colour applyCircles(const Intersection* intersect)
{
	Point p = (intersect->pos - intersect->material->offset) / intersect->material->size;

	int which = int(floorf(sqrtf(p.x*p.x + p.y*p.y + p.z*p.z))) & 1;

	return (which ? intersect->material->diffuse : intersect->material->diffuse2);
}


// apply computed wood grain texture
Colour applyWood(const Intersection* intersect)
{
	Point p = (intersect->pos - intersect->material->offset) / intersect->material->size;

	// squiggle up where the point is
	p = { p.x * cosf(p.y * 0.996f) * sinf(p.z * 1.023f), cosf(p.x) * p.y * sinf(p.z * 1.211f), cosf(p.x * 1.473f) * cosf(p.y * 0.795f) * p.z };

	int which = int(floorf(sqrtf(p.x*p.x + p.y*p.y + p.z*p.z))) & 1;

	return (which ? intersect->material->diffuse : intersect->material->diffuse2);
}
