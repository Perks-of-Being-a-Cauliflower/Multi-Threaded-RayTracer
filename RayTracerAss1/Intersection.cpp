/*  The following code is a VERY heavily modified from code originally sourced from:
	Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
	It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

#include "Intersection.h"

// test to see if collision between ray and a sphere happens before time t (equivalent to distance)
// updates closest collision time (/distance) if collision occurs
// see: http://en.wikipedia.org/wiki/Line-sphere_intersection
// see: http://www.codermind.com/articles/Raytracer-in-C++-Part-I-First-rays.html
// see: Step 8 of http://meatfighter.com/juggler/ 
// this code make heavy use of constant term removal due to ray always being a unit vector (i.e. normalised)
bool isSphereIntersected(const Sphere* s, const Ray* r, float* t)
{
    // Intersection of a ray and a sphere, check the articles for the rationale
    Vector dist = s->pos - r->start;
    float B = r->dir * dist;
    float D = B * B - dist * dist + s->size * s->size;

	// if D < 0, no intersection, so don't try and calculate the point of intersection
	if (D < 0.0f) return false;

	// calculate both intersection times(/distances)
	float t0 = B - sqrtf(D);
    float t1 = B + sqrtf(D);

	// check to see if either of the two sphere collision points are closer than time parameter
    if ((t0 > EPSILON) && (t0 < *t))
    {
        *t = t0;
		return true;
    } 
	else if ((t1 > EPSILON) && (t1 < *t))
    {
        *t = t1;
		return true;
    }

	return false;
}


// test to see if collision between ray and a (axis-aligned) box happens before time t (equivalent to distance)
// updates closest collision time (/distance) if collision occurs
// see: https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
bool isBoxIntersected(const Box* b, const Ray* r, float* t)
{
	// calculate distances to each "close" and "far" face, check the article for the rationale
	Vector t0 = (b->p1 - r->start) / r->dir;
	Vector t1 = (b->p2 - r->start) / r->dir;

	// determine which of t0 and t1 components are closest / furthest
	Vector tsmaller = { std::fmin(t0.x, t1.x), std::fmin(t0.y, t1.y), std::fmin(t0.z, t1.z) };
	Vector tbigger = { std::fmax(t0.x, t1.x), std::fmax(t0.y, t1.y), std::fmax(t0.z, t1.z) };

	// determine closest/furthest distance from x, y, and z components
	float tmin = std::fmax(tsmaller.x, std::fmax(tsmaller.y, tsmaller.z));
	float tmax = std::fmin(tbigger.x, std::fmin(tbigger.y, tbigger.z));

	// if closest distance is larger than furthest distance, exit
	if (tmin >= tmax) return false;

	// check to see if the closest collision point is closer than the time parameter
	if ((tmin > EPSILON) && (tmin < *t))
	{
		*t = tmin;
		return true;
	}

	return false;
}


// calculate collision normal, viewProjection, object's material, and test to see if inside collision object
void calculateIntersectionResponse(const Scene* scene, const Ray* viewRay, Intersection* intersect)
{
	switch (intersect->objectType)
	{
	case Intersection::SPHERE:
		intersect->normal = normalise(intersect->pos - intersect->sphere->pos);
		intersect->material = &scene->materialContainer[intersect->sphere->materialId];
		break;
	case Intersection::BOX:
		{		
			Vector size = intersect->box->p2 - intersect->box->p1;
			Vector centre = (intersect->box->p2 + intersect->box->p1) * 0.5f;
			Point diff = intersect->pos - centre;

			if (fabs(diff.x) / size.x > fabs(diff.y) / size.y && fabs(diff.x) / size.x > fabs(diff.z) / size.z)
			{
				intersect->normal = Vector { diff.x >= 0 ? 1.0f : -1.0f, 0, 0 };
			}
			else if (fabs(diff.y) / size.y > fabs(diff.x) / size.x && fabs(diff.y) / size.y > fabs(diff.z) / size.z)
			{
				intersect->normal = Vector{ 0, diff.y >= 0 ? 1.0f : -1.0f, 0 };
			}
			else
			{
				intersect->normal = Vector { 0, 0, diff.z >= 0 ? 1.0f : -1.0f };
			}
			intersect->normal = normalise(intersect->normal);
		}

		intersect->material = &scene->materialContainer[intersect->box->materialId];
		break;
	}

	// calculate view projection
	intersect->viewProjection = viewRay->dir * intersect->normal; 

	// detect if we are inside an object (needed for refraction)
	intersect->insideObject = (intersect->normal * viewRay->dir > 0.0f);

	// if inside an object, reverse the normal
    if (intersect->insideObject)
    {
        intersect->normal = intersect->normal * -1.0f;
    }
}


// test to see if collision between ray and any object in the scene
// updates intersection structure if collision occurs
bool objectIntersection(const Scene* scene, const Ray* viewRay, Intersection* intersect)
{
	// set default distance to be a long long way away
    float t = MAX_RAY_DISTANCE;

	// no intersection found by default
	intersect->objectType = Intersection::NONE;

	// search for sphere collisions, storing closest one found
    for (unsigned int i = 0; i < scene->numSpheres; ++i)
    {
        if (isSphereIntersected(&scene->sphereContainer[i], viewRay, &t))
        {
			intersect->objectType = Intersection::SPHERE;
			intersect->sphere = &scene->sphereContainer[i];
        }
    }

	// search for box collisions, storing closest one found
	for (unsigned int i = 0; i < scene->numBoxes; ++i)
	{
		if (isBoxIntersected(&scene->boxContainer[i], viewRay, &t))
		{
			intersect->objectType = Intersection::BOX;
			intersect->box = &scene->boxContainer[i];
		}
	}

	// nothing detected, return false
	if (intersect->objectType == Intersection::NONE)
	{
		return false;
	}

	// calculate the point of the intersection
	intersect->pos = viewRay->start + viewRay->dir * t;

	return true;
}
