/*  The following code is a VERY heavily modified from code originally sourced from:
Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

#include "Lighting.h"
#include "Colour.h"
#include "Intersection.h"
#include "Texturing.h"

// test to see if light ray collides with any of the scene's objects
// short-circuits when first intersection discovered, because no matter what the object will be in shadow
bool isInShadow(const Scene* scene, const Ray* lightRay, const float lightDist)
{
	float t = lightDist;

	// search for sphere collision
	for (unsigned int i = 0; i < scene->numSpheres; ++i)
	{
		if (isSphereIntersected(&scene->sphereContainer[i], lightRay, &t))
		{
			return true;
		}
	}

	// search for box collision
	for (unsigned int i = 0; i < scene->numBoxes; ++i)
	{
		if (isBoxIntersected(&scene->boxContainer[i], lightRay, &t))
		{
			return true;
		}
	}

	// not in shadow
	return false;
}


// apply diffuse lighting with respect to material's colouring
Colour applyDiffuse(const Ray* lightRay, const Light* currentLight, const Intersection* intersect)
{
	Colour output;

	switch (intersect->material->type)
	{
	case Material::GOURAUD:
		output = intersect->material->diffuse;
		break;
	case Material::CHECKERBOARD:
		output = applyCheckerboard(intersect);
		break;
	case Material::CIRCLES:
		output = applyCircles(intersect);
		break;
	case Material::WOOD:
		output = applyWood(intersect);
		break;
	}

	float lambert = lightRay->dir * intersect->normal;

	return lambert * currentLight->intensity * output;
}


// Blinn 
// The direction of Blinn is exactly at mid point of the light ray and the view ray. 
// We compute the Blinn vector and then we normalize it then we compute the coeficient of blinn
// which is the specular contribution of the current light.
Colour applySpecular(const Ray* lightRay, const Light* currentLight, const float fLightProjection, const Ray* viewRay, const Intersection* intersect)
{
	Vector blinnDir = lightRay->dir - viewRay->dir;
	float blinn = invsqrtf(blinnDir.dot()) * std::max(fLightProjection - intersect->viewProjection, 0.0f);
	blinn = powf(blinn, intersect->material->power);

	return blinn * intersect->material->specular * currentLight->intensity;
}


// apply diffuse and specular lighting contributions for all lights in scene taking shadowing into account
Colour applyLighting(const Scene* scene, const Ray* viewRay, const Intersection* intersect)
{
	// colour to return (starts as black)
	Colour output(0.0f, 0.0f, 0.0f);

	// same starting point for each light ray
	Ray lightRay = { intersect->pos };

	// loop through all the lights
	for (unsigned int j = 0; j < scene->numLights; ++j)
	{
		// get reference to current light
		const Light* currentLight = &scene->lightContainer[j];

		// light ray direction need to equal the normalised vector in the direction of the current light
		// as we need to reuse all the intermediate components for other calculations, 
		// we calculate the normalised vector by hand instead of using the normalise function
		lightRay.dir = currentLight->pos - intersect->pos;
		float angleBetweenLightAndNormal = lightRay.dir * intersect->normal;

		// skip this light if it's behind the object (ie. both light and normal pointing in the same direction)
		if (angleBetweenLightAndNormal <= 0.0f)
		{
			continue;
		}

		// distance to light from intersection point (and it's inverse)
		float lightDist = sqrtf(lightRay.dir.dot());
		float invLightDist = 1.0f / lightDist;

		// light ray projection
		float lightProjection = invLightDist * angleBetweenLightAndNormal;

		// normalise the light direction
		lightRay.dir = lightRay.dir * invLightDist;

		// only apply lighting from this light if not in shadow of some other object
		if (!isInShadow(scene, &lightRay, lightDist))
		{
			// add diffuse lighting from colour / texture
			output += applyDiffuse(&lightRay, currentLight, intersect);

			// add specular lighting
			output += applySpecular(&lightRay, currentLight, lightProjection, viewRay, intersect);
		}
	}

	return output;
}
