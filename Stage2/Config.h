/*  The following code is a VERY heavily modified from code originally sourced from:
	Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
	It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

// YOU SHOULD _NOT_ NEED TO MODIFY THIS FILE

// THIS FILE DOES NOT FOLLOW THE SAME FORMATTING CONVENTIONS AS THE REST OF THIS PROJECT

#ifndef __CONFIG_H
#define __CONFIG_H

// This is a simple config file parser. It does what we need no more no less.

#include "Primitives.h"
#include "SceneObjects.h"
#include "Colour.h"
#include "SimpleString.h"
#pragma warning( push )
#pragma warning( disable : 4512 ) // assignment operator cannot be generated because of the const member. We don't need one.

// Some code
class Config {
private:
    void* m_pVariables;
    void* m_pSections;
    const SimpleString m_sFileName;
    SimpleString m_sCurrentSection;
    bool m_bLoaded;
public:
    // When the variable called "sName" doesn't exit, you will get "default" 
    bool GetByNameAsBoolean(const SimpleString  & sName, bool bDefault) const;
    double GetByNameAsFloat(const SimpleString & sName, double fDefault) const;
    const SimpleString &GetByNameAsString(const SimpleString  &sName, const SimpleString  & sDefault) const;
    long GetByNameAsInteger(const SimpleString  &sName, long lDefault) const;
    Vector GetByNameAsVector(const SimpleString &sName, const Vector& vDefault) const;
    Point GetByNameAsPoint(const SimpleString &sName, const Point& ptDefault) const;
	Colour GetByNameAsFloatOrColour(const SimpleString& sName, double fDefaut) const;
    
    // SetSection will return -1 when the section wasn't found. 
    int SetSection(const SimpleString &sName);
    ~Config();
    Config(const SimpleString &sFileName);
};

#pragma warning( pop ) 
#endif //__CONFIG_H
