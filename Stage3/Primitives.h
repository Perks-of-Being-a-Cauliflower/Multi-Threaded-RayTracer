/*  The following code is a VERY heavily modified from code originally sourced from:
	Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
	It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

#ifndef __PRIMITIVES_H
#define __PRIMITIVES_H

#include <cmath>

#include "Constants.h"


// points consist of three coordinates and represent a point in 3d space
typedef struct Point 
{
	float x, y, z;

	inline float length()
	{
		return sqrtf(x * x + y * y + z * z);
	}
} Point;


// vectors consist of three coordinates and represent a direction from (an implied) origin
typedef struct Vector 
{
	float x, y, z;

    Vector& operator += (const Vector& v2) 
	{
	    this->x += v2.x;
        this->y += v2.y;
        this->z += v2.z;
	    return *this;
    }

	inline float dot() const 
	{
		return x * x + y * y + z * z;
	}
} Vector;


// rays are cast from a starting point in a direction
typedef struct Ray
{
	Point start;
	Vector dir;
} Ray;


// ---- Point specific functions/operators ----

// point + vector (produces a point)
inline Point operator + (const Point& p, const Vector& v) 
{
	Point p2 = { p.x + v.x, p.y + v.y, p.z + v.z };
	return p2;
}

// point - vector (produces a point)
inline Point operator - (const Point& p, const Vector& v)
{
	Point p2 = { p.x - v.x, p.y - v.y, p.z - v.z };
	return p2;
}

// point * float (each component of the point is multiplied by the float)
inline Point operator * (const Point& p, float c)
{
	Point p2 = { p.x * c, p.y * c, p.z * c };
	return p2;
}

// point / float (each component of the point is divided by the float)
inline Point operator / (const Point& p, float c)
{
	Point p2 = { p.x / c, p.y / c, p.z / c };
	return p2;
}


// ---- Vector specific functions/operators ----

// vector + vector
inline Vector operator + (const Vector& v1, const Vector& v2)
{
	Vector v = { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
	return v;
}

// vector / vector
inline Vector operator / (const Vector& v1, const Vector& v2)
{
	Vector v = { v1.x / v2.x, v1.y / v2.y, v1.z / v2.z };
	return v;
}

// point + point (produces a vector)
inline Vector operator + (const Point& p1, const Point& p2)
{
	Vector v = { p1.x + p2.x, p1.y + p2.y, p1.z + p2.z };
	return v;
}

// point - point (produces a vector)
inline Vector operator - (const Point& p1, const Point& p2)
{
	Vector v = { p1.x - p2.x, p1.y - p2.y, p1.z - p2.z };
	return v;
}

// vector * float (each component of the vector is multipled by the float)
inline Vector operator * (const Vector& v, float c)
{
	Vector v2 = { v.x * c, v.y * c, v.z * c };
	return v2;
}

// vector - vector
inline Vector operator - (const Vector& v1, const Vector& v2)
{
	Vector v = { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
	return v;
}

// vector * vector (dot-product, produces a single float)
inline float operator * (const Vector& v1, const Vector& v2 ) 
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// helper function
inline float invsqrtf(const float& x) 
{ 
	return 1.0f / sqrtf(x); 
}

// normalise the vector (ie. make the vector's magnitude equal to 1)
// not to be confused with the normal vector of a surface/intersection
inline Vector normalise(const Vector& x)
{
	return x * invsqrtf(x.dot());
}

// vector cross product
inline Vector cross(const Vector& v1, const Vector& v2)
{
	Vector v = { v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x };
	return v;
}


#endif //__PRIMITIVES_H
