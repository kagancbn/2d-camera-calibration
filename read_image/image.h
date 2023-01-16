
#include <string.h>

// image.h
typedef struct image {
	int w;
	int h;
	int c;
	unsigned char* data;
} image;

typedef struct gradient {
	int w;
	int h;
	double* direction;
	double* magnititude;
} gradient;

typedef struct hough {
	int w;
	int h;
	int* data;
} hough;

typedef struct line {
	int max;
	int theta;
	int distance;
}line;
typedef struct point
{
	int id;
	int X;
	int Y;
}point;



image load_image(const char* filename);
image make_image(int w, int h, int c);
image make_empty_image(int w, int h, int c);
image RgbToGray(image input);
image Smoothing(image input, int filterSize);

gradient Gradient(image input, gradient grad);

image CannyEdge(gradient grad);

hough HoughLine(image BinaryEdgeImage, hough HoughSpace);
void DrawSelectedLines(hough HoughSpace, image im, int& numberofline, line*& lpointer);
void IntersectionPoint(int number_of_lines, line* line, hough HoughSpace, point*& p, int& pointnumber, image im);
int parametricIntersect(int r1, int t1, int r2, int t2, int& x, int& y);

bool autoCalibVerticalImage(point* points, int numofDetectedP, point*& autoDetectPts, image im);
void autoCalibImage(point* points, int numofDetectedP, point*& autoDetectPts, image im);

void Compute_Projection_Matrix(int point_count, float* world_points, point* image_points, float* A);
void Reconstruct(int numofPoint, float* imageArray, float* P, float* worldArray);

