/*  The following code is a VERY heavily modified from code originally sourced from:
Ray tracing tutorial of http://www.codermind.com/articles/Raytracer-in-C++-Introduction-What-is-ray-tracing.html
It is free to use for educational purpose and cannot be redistributed outside of the tutorial pages. */

#define TARGET_WINDOWS

#pragma warning(disable: 4996)
#include <stdio.h>
#include "Timer.h"
#include "Primitives.h"
#include "Scene.h"
#include "Lighting.h"
#include "Intersection.h"
#include "ImageIO.h"
#include <windows.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

// buffer size
unsigned int buffer[(MAX_WIDTH) * (MAX_HEIGHT)];
// global storage for samples rendered
int samplesRenderedTotal = 0;
// global timer
int totalTime = 0;
// set number of runs
int times = 1;
// handle to prevent samplesRenderedTotal from being accessed by more than one thread at a time
HANDLE sum_mutex;

// thread data struct with variables needed to run render code through seperate threads
struct ThreadData {
	// unique thread id
	unsigned int threadID;
	// current scene
	Scene *scene;
	// width
	int width;
	// height
	int height;
	// samples
	int aaLevel;
	// test mode check
	bool testMode;
	// output variable
	unsigned int* out;
	// start x location for each thread
	int startx;
	// start y location for each thread
	int starty;
	// total number of threads
	int numThreads;
	// check if want image to be colourised
	bool colourise;

};




// reflect the ray from an object
Ray calculateReflection(const Ray* viewRay, const Intersection* intersect)
{
	// reflect the viewRay around the object's normal
	Ray newRay = { intersect->pos, viewRay->dir - (intersect->normal * intersect->viewProjection * 2.0f) };

	return newRay;
}


// refract the ray through an object
Ray calculateRefraction(const Ray* viewRay, const Intersection* intersect, float* currentRefractiveIndex)
{
	// change refractive index depending on whether we are in an object or not
	float oldRefractiveIndex = *currentRefractiveIndex;
	*currentRefractiveIndex = intersect->insideObject ? DEFAULT_REFRACTIVE_INDEX : intersect->material->density;

	// calculate refractive ratio from old index and current index
	float refractiveRatio = oldRefractiveIndex / *currentRefractiveIndex;

	// Here we take into account that the light movement is symmetrical from the observer to the source or from the source to the oberver.
	// We then do the computation of the coefficient by taking into account the ray coming from the viewing point.
	float fCosThetaT;
	float fCosThetaI = fabsf(intersect->viewProjection);

	// glass-like material, we're computing the fresnel coefficient.
	if (fCosThetaI >= 1.0f)
	{
		// In this case the ray is coming parallel to the normal to the surface
		fCosThetaT = 1.0f;
	}
	else
	{
		float fSinThetaT = refractiveRatio * sqrtf(1 - fCosThetaI * fCosThetaI);

		// Beyond the angle (1.0f) all surfaces are purely reflective
		fCosThetaT = (fSinThetaT * fSinThetaT >= 1.0f) ? 0.0f : sqrtf(1 - fSinThetaT * fSinThetaT);
	}

	// Here we compute the transmitted ray with the formula of Snell-Descartes
	Ray newRay = { intersect->pos, (viewRay->dir + intersect->normal * fCosThetaI) * refractiveRatio - (intersect->normal * fCosThetaT) };

	return newRay;
}


// follow a single ray until it's final destination (or maximum number of steps reached)
Colour traceRay(const Scene* scene, Ray viewRay)
{
	Colour output(0.0f, 0.0f, 0.0f); 								// colour value to be output
	float currentRefractiveIndex = DEFAULT_REFRACTIVE_INDEX;		// current refractive index
	float coef = 1.0f;												// amount of ray left to transmit
	Intersection intersect;											// properties of current intersection

																	// loop until reached maximum ray cast limit (unless loop is broken out of)
	for (int level = 0; level < MAX_RAYS_CAST; ++level)
	{
		// check for intersections between the view ray and any of the objects in the scene
		// exit the loop if no intersection found
		if (!objectIntersection(scene, &viewRay, &intersect)) break;

		// calculate response to collision: ie. get normal at point of collision and material of object
		calculateIntersectionResponse(scene, &viewRay, &intersect);

		// apply the diffuse and specular lighting 
		if (!intersect.insideObject) output += coef * applyLighting(scene, &viewRay, &intersect);

		// if object has reflection or refraction component, adjust the view ray and coefficent of calculation and continue looping
		if (intersect.material->reflection)
		{
			viewRay = calculateReflection(&viewRay, &intersect);
			coef *= intersect.material->reflection;
		}
		else if (intersect.material->refraction)
		{
			viewRay = calculateRefraction(&viewRay, &intersect, &currentRefractiveIndex);
			coef *= intersect.material->refraction;
		}
		else
		{
			// if no reflection or refraction, then finish looping (cast no more rays)
			return output;
		}
	}

	// if the calculation coefficient is non-zero, read from the environment map
	if (coef > 0.0f)
	{
		Material& currentMaterial = scene->materialContainer[scene->skyboxMaterialId];

		output += coef * currentMaterial.diffuse;
	}

	return output;
}


// render scene at given width and height and anti-aliasing level
void render(Scene* scene, const int width, const int height, const int aaLevel, bool testMode, unsigned int threadID, unsigned int* out, int startx, int starty, int numThreads, bool colourise)
{
	// angle between each successive ray cast (per pixel, anti-aliasing uses a fraction of this)
	const float dirStepSize = 1.0f / (0.5f * width / tanf(PIOVER180 * 0.5f * scene->cameraFieldOfView));


	// count of samples rendered
	unsigned int samplesRendered = 0;

	// loop through all the pixels
	// loop is calculated according to the height of the thread
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
				Colour output(0.0f, 0.0f, 0.0f);

				// calculate multiple samples for each pixel
				const float sampleStep = 1.0f / aaLevel, sampleRatio = 1.0f / (aaLevel * aaLevel);

				// loop through all sub-locations within the pixel
				// start x and start y are added as offsets to assign different sections to different threads
				for (float fragmentx = float(x + startx); fragmentx < x + startx + 1.0f; fragmentx += sampleStep)
				{
					for (float fragmenty = float(y + starty); fragmenty < (y + starty) + 1.0f; fragmenty += sampleStep)
					{
						// direction of default forward facing ray
						Vector dir = { fragmentx * dirStepSize, fragmenty * dirStepSize, 1.0f };

						// rotated direction of ray
						Vector rotatedDir = {
							dir.x * cosf(scene->cameraRotation) - dir.z * sinf(scene->cameraRotation),
							dir.y,
							dir.x * sinf(scene->cameraRotation) + dir.z * cosf(scene->cameraRotation) };

						// view ray starting from camera position and heading in rotated (normalised) direction
						Ray viewRay = { scene->cameraPosition, normalise(rotatedDir) };

						// follow ray and add proportional of the result to the final pixel colour
						output += sampleRatio * traceRay(scene, viewRay);

						// count this sample
						samplesRendered++;
					}
				}

				if (!testMode)
				{

					if (!colourise) {
						// store saturated final colour value in image buffer
						*out++ = output.convertToPixel(scene->exposure);
					}
					else {
						// if the leftmost bit is a "1" then add a red tint according to its thread ID
						if ((threadID % 7) & 4) {
							output.red *= (float)threadID / numThreads;
						}
						// if the middle bit is a "1" then add a green tint according to its thread ID
						if ((threadID % 7) & 2) {
							output.green *= (float)threadID / numThreads;
						}
						// if the rightmost bit is a "1" then add a blue tint according to its thread ID
						if ((threadID % 7) & 1) {
							output.blue *= (float)threadID / numThreads;
						}
						// store saturated final colour value in image buffer
						*out++ = output.convertToPixel(scene->exposure);
					}
				}
				else
				{
					// store white in image buffer (with multiple threads this should store a grey based on the thread number)
					*out++ = Colour(1, 1, 1).convertToPixel(threadID);
				}
			}
		}
	// get access to mutex to ensure samplesRenderedTotal is not currently being accessed.
	WaitForSingleObject(sum_mutex, INFINITE);
	// update the samples rendered
	samplesRenderedTotal += samplesRendered;
	// release mutex to allow other threads to access the global samples variable
	ReleaseMutex(sum_mutex);
}

// create a start routine to run the selected function from the different threads and pass through the thread values
DWORD __stdcall THREAD_START_ROUTINE(void* threadid)
{
	// cast the threadid data into a new ThreadData struct
	ThreadData* data = (ThreadData*)threadid;
	// call render with the unqiue thread values that have already been partitioned
	render(data->scene, data->width, data->height, data->aaLevel, data->testMode, data->threadID, data->out, data->startx, data->starty, data->numThreads, data->colourise);	// raytrace scene
	// when complete exit thread
	ExitThread(NULL);
}


// read command line arguments, render, and write out BMP file
int main(int argc, char* argv[])
{
	// set width
	int width = 1024;
	// set height
	int height = 1024;
	// set samples
	int samples = 1;

	// rendering options

	// set if testmode
	bool testMode = false;
	// set if colourise
	bool colourise = false;				
	unsigned int blockSize = 0;		// currently unused
	// set out to be the size of the buffer
	unsigned int *out = buffer;
	// set number of threads
	unsigned int threads = 8; 	
	// create the mutex
	sum_mutex = CreateMutex(NULL, false, NULL);

	// default input / output filenames
	const char* inputFilename = "Scenes/cornell.txt";

	char outputFilenameBuffer[1000];
	char* outputFilename = outputFilenameBuffer;

	// do stuff with command line args
	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-size") == 0)
		{
			width = atoi(argv[++i]);
			height = atoi(argv[++i]);
		}
		else if (strcmp(argv[i], "-samples") == 0)
		{
			samples = atoi(argv[++i]);
		}
		else if (strcmp(argv[i], "-input") == 0)
		{
			inputFilename = argv[++i];
		}
		else if (strcmp(argv[i], "-output") == 0)
		{
			outputFilename = argv[++i];
		}
		else if (strcmp(argv[i], "-runs") == 0)
		{
			times = atoi(argv[++i]);
		}
		else if (strcmp(argv[i], "-threads") == 0)
		{
			threads = atoi(argv[++i]);
		}
		else if (strcmp(argv[i], "-colourise") == 0)
		{
			colourise = true;
		}
		else if (strcmp(argv[i], "-blockSize") == 0)
		{
			blockSize = atoi(argv[++i]);
		}
		else if (strcmp(argv[i], "-testMode") == 0)
		{
			testMode = true;
		}
		else
		{
			fprintf(stderr, "unknown argument: %s\n", argv[i]);
		}
	}

	// nasty (and fragile) kludge to make an ok-ish default output filename (can be overriden with "-output" command line option)
	sprintf(outputFilenameBuffer, "Outputs/%s_%dx%dx%d_%s.bmp", (strrchr(inputFilename, '/') + 1), width, height, samples, (strrchr(argv[0], '\\') + 1));

	// read scene file
	Scene scene;
	if (!init(inputFilename, scene))
	{
		fprintf(stderr, "Failure when reading the Scene file.\n");
		return -1;
	}

	for (int i = 0; i < times; i++)
	{
		Timer timer;

		// created an array of HANDLEs dynamically
		HANDLE* threadsHandle = new HANDLE[threads];

		// created an array of ThreadDatas dynamically
		ThreadData* data = new ThreadData[threads];

		// offset counter for threads larger than half the height
		unsigned int offsetcounter = 0;

		// loop through each thread
		for (int i = 0; i < threads; i++) {
			// set the initial height to divide each thread into the heigh evenly
			unsigned int heightofchunk = height / threads;
			// offsetofchunk is used if threads is greater than height/2 to help evenly divide the workload around each thread rather than have the last thread handle a large chunk
			unsigned int offsetofchunk = height / threads;
			// temporary out value which is different depending if the threads is greater than height/2
			unsigned int* outvalue = out + i * width * offsetofchunk;
			// set the initial offset incase the threads does not divide evenly into the height
			unsigned int offset = 0;


			// check if threads does not divide evenly and then check if it is also the final thread to be run
			if (height % threads != 0 && i == threads-1) {
				// set the offset to be the excess
				offset = height % threads;
			}
			
			// check if the threads is greater than the height/2
			if (threads > height / 2) {
				// make sure the offset is 0 since the workload will be split differently
				offset = 0;
				//increase the height and offset of the chunk so they take twice as much work 
				heightofchunk++;
				offsetofchunk++;
				// set the new outvalue to calculate based of the new chunk height
				outvalue = out + i * width * offsetofchunk;

				// check that the last x threads do less work. e.g 513 threads will divide the load so the first 511 (1024 - 513) threads do 2 lines and the last two split the remaining work
				if (i >= (threads - (threads -(height % threads)))) {
					// the last threads do 1 line each
					heightofchunk = height / threads;
					// check to make sure the out and starty locations are correct
					if (i >= (threads - (threads - (height % threads) - 1))) {
						// increase offset counter
						offsetcounter++;
						// set new out value so it accounts for the new height of the last chunks
						outvalue = (out + (i - offsetcounter) * width * offsetofchunk) + (offsetcounter * (width * heightofchunk));
					}
				}
			}
			
			// the threads id
			data[i].threadID = i;
			// the current scene
			data[i].scene = &scene;
			// the specified width
			data[i].width = width;
			// the specified height which is sometimes increased by the offset to account for threads not dividing into height
			data[i].height = heightofchunk + offset;
			// the specified number of samples
			data[i].aaLevel = samples;
			// if to colourise or not
			data[i].colourise = colourise;
			// if run on testmode
			data[i].testMode = testMode;
			// the total number of threads to calculate
			data[i].numThreads = threads;
			// the starting position for the x coordinate
			data[i].startx = -(width/2);
			// the starting position for the y coordinate which needs to account for how the workload is divided when threads > height/2
			data[i].starty = -(height / 2) + ((i - offsetcounter) * offsetofchunk) + offsetcounter;
			// the out value
			data[i].out = outvalue;

			// start a new thread with the above values and run the THREAD_START_ROUTINE function
			threadsHandle[i] = CreateThread(NULL, 0, THREAD_START_ROUTINE, &data[i], 0, NULL);
			




		}
		// wait for each thread to complete before finishing
		for (int i = 0; i < threads; i++) {
			WaitForSingleObject(threadsHandle[i], INFINITE);
		}
		// clear the memory of structures and arrarys
		delete[] data;
		delete[] threadsHandle;

		
		timer.end();								// record end time
		totalTime += timer.getMilliseconds();		// record total time taken
	}
	// output timing information (times run and average)
	printf("rendered %d samples, average time taken (%d run(s)): %ums\n", samplesRenderedTotal/times, times, totalTime / times);


	// output BMP file
	write_bmp(outputFilename, buffer, width, height, width);
}
