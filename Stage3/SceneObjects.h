/*  The following code is a VERY heavily modified from code originally sourced from:
	Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
	It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

#ifndef __SCENE_OBJECTS_H
#define __SCENE_OBJECTS_H

#include "Colour.h"
#include "Primitives.h"


// material
typedef struct Material
{
	// type of colouring/texturing
	enum { GOURAUD, CHECKERBOARD, CIRCLES, WOOD } type;

	Colour diffuse;				// diffuse colour
	Colour diffuse2;			// second diffuse colour, only for checkerboard types

	Vector offset;				// offset of generated texture
	float size;					// size of generated texture

	Colour specular;			// colour of specular lighting
	float power;				// power of specular reflection

	float reflection;			// reflection amount
	float refraction;			// refraction amount
	float density;				// density of material (affects amount of defraction)
} Material;


// light object
typedef struct Light
{
	Point pos;					// location
	Colour intensity;			// brightness and colour
} Light;


// sphere object
typedef struct Sphere
{
	Point pos;					// a point on the plane
	float size;					// radius of sphere
	unsigned int materialId;	// material id
} Sphere;


// (axis-aligned) (bounding) box object
typedef struct Box
{
	Point p1, p2;				// two points to define opposite corners of the box
	unsigned int materialId;	// material id
} Box;


#endif // __SCENE_OBJECTS_H
