/*  The following code is a VERY heavily modified from code originally sourced from:
	Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
	It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

#ifndef __INTERSECTION_H
#define __INTERSECTION_H

#include "Scene.h"
#include "SceneObjects.h"

// all pertinant information about an intersection of a ray with an object
typedef struct Intersection
{
	enum { NONE, SPHERE, BOX } objectType;	// type of object intersected with

	Point pos;											// point of intersection
	Vector normal;										// normal at point of intersection
	float viewProjection;								// view projection 
	bool insideObject;									// whether or not inside an object

	Material* material;									// material of object

	// object collided with
	union 
	{
		struct Sphere* sphere;
		struct Box* box;
	};
} Intersection;

// test to see if collision between ray and a sphere happens before time t (equivalent to distance)
// updates closest collision time (/distance) if collision occurs
bool isSphereIntersected(const Sphere* s, const Ray* r, float* t);

// test to see if collision between ray and a box happens before time t (equivalent to distance)
// updates closest collision time (/distance) if collision occurs
bool isBoxIntersected(const Box* b, const Ray* r, float* t);

// calculate collision normal, viewProjection, object's material, and test to see if inside collision object
void calculateIntersectionResponse(const Scene* scene, const Ray* viewRay, Intersection* intersect); 

// test to see if collision between ray and any object in the scene
// updates intersection structure if collision occurs
bool objectIntersection(const Scene* scene, const Ray* viewRay, Intersection* intersect);

#endif // __INTERSECTION_H
