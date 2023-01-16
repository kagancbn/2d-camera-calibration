#include "image.h"
#include <math.h>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/include/stb_image.h"

#include <Eigen/Dense>

#define PI 3.141592

#define weak 175 
#define strong 255

#define minvote 50				//Min vote value in houghspace it can be im.w* im.h * 0.0004
#define number_of_lines 75		//Max number of detecting lines

#define distQuantize 7			// Max distance between nearest two line in hough space (hough space height is w*h*2)
// smaller value more robust
#define thetaQuantize 4		// Max angle between nearest two line in hough space (houghspace w is 180)

#define number_of_points 200	// Max number of detecting points

using namespace Eigen;

image load_image(const char* filename)
{
	int w, h, c;
	int channel = 3;
	//w = width, h = height, c = # 8 - bit components per pixel ...
	unsigned char* data = stbi_load(filename, &w, &h, &c, channel);    // without OpenCV

	if (!data) {
		exit(EXIT_FAILURE);
	}

	image out;
	out.data = data;
	out.h = h;
	out.w = w;
	out.c = c;
	return out;
}//load_image

image make_image(int w, int h, int c)
{
	image out;
	out.w = w;
	out.h = h;
	out.c = c;
	out.data = new unsigned char[w * h * c];
	for (int i = 0; i < w * h * c; i++)
		out.data[i] = 0;
	return out;
}
image make_empty_image(int w, int h, int c)
{
	image out;
	out.w = w;
	out.h = h;
	out.c = c;
	return out;
}
image RgbToGray(image input)
{
	image out = make_image(input.w, input.h, 1);

	for (int row = 0; row < out.h; row++)
		for (int col = 0; col < out.w; col++) {
			out.data[row * out.w + col] = input.data[row * input.w * 3 + col * 3] * 0.29 + input.data[row * input.w * 3 + col * 3 + 1] * 0.58 + input.data[row * input.w * 3 + col * 3 + 2] * 0.11;
		}
	return out;
}
image Smoothing(image input, int kernelSize)
{
	int out_w = (input.w - kernelSize + 1);
	int out_h = (input.h - kernelSize + 1);

	image out = make_image(out_w, out_h, 1);

	int kernel3x[9] = { 1,2,1,
						2,4,2,
						1,2,1 };

	int kernel5x[25] = { 1,4,7,4,1,
						4,16,26,16,4,
						7,26,41,26,7,
						4,16,26,16,4,
						1,4,7,4,1 };
	int* kernel;
	int div;
	switch (kernelSize)
	{
	case 5:
		kernel = kernel5x;
		div = 273;
		break;
	default:
		kernel = kernel3x;
		div = 16;
		break;
	}

	for (int row = 0; row < out.h; row++)
		for (int col = 0; col < out.w; col++) {
			int sum = 0;
			for (int krow = 0; krow < kernelSize; krow++)
				for (int kcol = 0; kcol < kernelSize; kcol++) {

					sum += input.data[(row + krow) * input.w + (col + kcol)] * kernel[krow * kernelSize + kcol];
				}
			out.data[row * out.w + col] = (int)sum / div;
		}
	return out;
}

gradient Gradient(image input, gradient grad)
{
	int KernelX[9] = { 1,0,-1,2,0,-2,1,0,-1 };
	// horizontal direction sobel kernel (Gx)

	int KernelY[9] = { 1,2,1,0,0,0,-1,-2,-1 };
	// vertical direction sobel kernel(Gy)

	int kernelSize = 3;
	int padding = 1;

	grad.w = input.w - 2;
	grad.h = input.h - 2;
	grad.magnititude = new double[grad.h * grad.w];				// G|x| + G|y| gradient array
	grad.direction = new double[grad.h * grad.w];				// G|x| + G|y| gradient orientation array

	for (int i = 0; i < grad.h * grad.w; i++)
	{
		grad.magnititude[i] = 0;
		grad.direction[i] = 0;
	}

	for (int row = 0; row < grad.h; row++)
		for (int col = 0; col < grad.w; col++) {

			double sumx = 0, sumy = 0;
			double direction = 0;

			for (int m = 0; m < kernelSize; m++)
				for (int n = 0; n < kernelSize; n++) {

					sumx += input.data[(row + m) * input.w + (col + n)] * KernelX[m * kernelSize + n];
					sumy += input.data[(row + m) * input.w + (col + n)] * KernelY[m * kernelSize + n];
				}

			//Gradient magnititude calculation

			grad.magnititude[row * grad.w + col] = abs(sumx) + abs(sumy);				// G|x| + G|y| gradient magnititude array

			//Angle calculation

			direction = atan2(sumy, sumx) * 180 / PI;						 // arctan2(Gy,Gx);

			grad.direction[row * grad.w + col] = direction;

		}
	return grad;
}

void hysteresisThreshold(int pos, int width, int height, int* data)
{
	for (int i = -1; i < 2; i++)
		for (int j = -1; j < 2; j++) {
			if (pos + i * width + j > 0 && pos + i * width + j < width * height)
				if (data[pos + i * width + j] == weak) {
					// if strong pixel has a weak neighbour pixel
					data[pos + i * width + j] = strong;
					// sign the weak neighbour pixel as a strong value
					hysteresisThreshold(pos + i * width + j, width, height, data);
					// call function again with changed pixel pos
				}
		}
}
image CannyEdge(gradient grad)
{
	int width = grad.w;
	int height = grad.h;
	//------------Non-max supression-------------------

	int* nonmax = new int[width * height];
	for (int i = 0; i < width * height; i++)
		nonmax[i] = 0;

	for (int row = 1; row < grad.h - 1; row++)
		for (int col = 1; col < grad.w - 1; col++) {
			int	q = 0, r = 0;
			int Q = grad.direction[row * grad.w + col];
			if ((0 <= Q && Q < 22.5) || (337.5 <= Q && Q <= 360) || (157.5 <= Q && Q < 202.5) || (0 > Q && Q > -22.5) || (-157.5 >= Q && Q > -202.5) || (-337.5 >= Q && Q >= -360))	// 0
			{
				q = grad.magnititude[row * grad.w + col - 1];					// left-pixel
				r = grad.magnititude[row * grad.w + col + 1];					//right-pixel
			}
			else if ((22.5 <= Q && Q < 67.5) || (202.5 <= Q && Q < 247.5) || (-112.5 >= Q && Q > -157.5) || (-292.5 >= Q && Q > -337.5))		// 45
			{
				q = grad.magnititude[(row - 1) * grad.w + col - 1];
				r = grad.magnititude[(row + 1) * grad.w + col + 1];

			}
			else if ((67.5 <= Q && Q < 112.5) || (247.5 <= Q && Q < 292.5) || (-67.5 >= Q && Q > -112.5) || (-247.5 >= Q && Q > -292.5))	// 90
			{
				q = grad.magnititude[(row - 1) * grad.w + col];						// up- pixel
				r = grad.magnititude[(row + 1) * grad.w + col];						// bottom- pixel
			}
			else if ((112.5 <= Q && Q < 157.5) || (292.5 <= Q && Q < 337.5) || (-22.5 >= Q && Q > -67.5) || (-202.5 >= Q && Q > -247.5))	// 135
			{
				q = grad.magnititude[(row + 1) * grad.w + col - 1];
				r = grad.magnititude[(row - 1) * grad.w + col + 1];

			}
			if (grad.magnititude[row * grad.w + col] > q && grad.magnititude[row * grad.w + col] > r) { // if grater then both neighboor keep it else supress it

				nonmax[row * grad.w + col] = grad.magnititude[row * grad.w + col];
			}
		}


	//------------------Double Thresholding---------------
	int max = 0;
	for (int row = 0; row < height; row++)
		for (int col = 0; col < width; col++)
			if (nonmax[row * width + col] > max) {
				max = nonmax[row * width + col];
			}
	int high_T = 0.3 * max; // high threshold value
	int low_T = 0.1 * max;	// low threshold value
	int strongCount = 0;
	int weakCount = 0;
	for (int row = 0; row < height; row++)
		for (int col = 0; col < width; col++) {
			if (nonmax[row * width + col] <= low_T) {
				nonmax[row * width + col] = 0;
			}
			else if (nonmax[row * width + col] >= high_T) {
				nonmax[row * width + col] = strong;
				strongCount++;
			}
			else {
				nonmax[row * width + col] = weak;
				weakCount++;
			}
		}
	//---------------Hysteresis Edge Tracking -----------------
	int* strongEdgePos = new int[strongCount];
	int* weakEdgePos = new int[weakCount];
	int strongEdgeIndex = 0;
	int weakEdgeIndex = 0;
	// indexing according to weak values and strong values
	for (int row = 0; row < height; row++)
		for (int col = 0; col < width; col++) {
			int pos = row * width + col;
			if (nonmax[row * width + col] == strong) {
				strongEdgePos[strongEdgeIndex] = pos;
				strongEdgeIndex++;
			}
			if (nonmax[row * width + col] == weak) {
				weakEdgePos[weakEdgeIndex] = pos;
				weakEdgeIndex++;
			}
		}
	for (int i = 0; i < strongCount; i++) {
		int pos = strongEdgePos[i];
		hysteresisThreshold(pos, width, height, nonmax);
		// If strong pixels's neighbours are weak turn them to strong 
	}
	for (int i = 0; i < weakCount; i++) {
		int pos = weakEdgePos[i];
		if (nonmax[pos] == weak)
			// If weak pixels have no strong neighbour pixel, remove the weaks
			nonmax[pos] = 0;
	}


	int HysWidth = width - 2;			//width and height is for nonmax image sizes
	int HysHeight = height - 2;			//it must be change for kernel pass
	int* Hysteresis = new int[HysWidth * HysHeight];
	for (int i = 0; i < HysHeight * HysWidth; i++)
		Hysteresis[i] = 0;
	for (int row = 1; row < height - 1; row++)
		for (int col = 1; col < width - 1; col++)
			Hysteresis[(row - 1) * HysWidth + (col - 1)] = nonmax[row * width + col];


	image out = make_image(HysWidth, HysHeight, 1);
	for (int i = 0; i < HysWidth * HysHeight; i++)
	{
		out.data[i] = Hysteresis[i];
	}
	delete[] Hysteresis;
	delete[] nonmax;
	delete[] strongEdgePos;
	delete[] weakEdgePos;
	return out;
}

hough HoughLine(image BinaryEdgeImage, hough HoughSpace) {
	HoughSpace.w = 180;
	HoughSpace.h = 2 * sqrt(BinaryEdgeImage.w * BinaryEdgeImage.w + BinaryEdgeImage.h * BinaryEdgeImage.h);
	// Height means distance. 2x for negatif values
	HoughSpace.data = new int[HoughSpace.w * HoughSpace.h]; // Accumulator array (votes)
	for (int i = 0; i < HoughSpace.w * HoughSpace.h; i++)
		HoughSpace.data[i] = 0;
	for (int row = 1; row < BinaryEdgeImage.h; row++)
		for (int col = 1; col < BinaryEdgeImage.w; col++)
			if (BinaryEdgeImage.data[row * BinaryEdgeImage.w + col] == 255)
				for (int theta = 0; theta < HoughSpace.w; theta++) {
					int distance = col * cos(theta * PI / 180) + row * sin(theta * PI / 180) + (HoughSpace.h / 2);
					// hough height/2 is for : negatif distance can't hold in 1d accumulator array
					HoughSpace.data[distance * HoughSpace.w + theta]++;
				}
	return HoughSpace;
}
void DrawSelectedLines(hough HoughSpace, image im, int& numberofline, line*& Line) {

	int* max = new int[number_of_lines];				//Max voted points array
	int* distance = new int[number_of_lines];			//Distance parameter array
	int* theta = new int[number_of_lines];				//Theta parameter array
	for (int i = 0; i < number_of_lines; i++)
		max[i] = -1, distance[i] = -1, theta[i] = -1;
	// ------------------------------finding max voted values in hough space---------------
	for (int row = 0; row < HoughSpace.h; row++)// distance indice d
		for (int col = 0; col < HoughSpace.w; col++) {//Theta indice 
			if (HoughSpace.data[row * HoughSpace.w + col] > max[number_of_lines - 1] &&
				HoughSpace.data[row * HoughSpace.w + col] > minvote) {

				for (int i = 1; i < number_of_lines; i++) {// 
					max[i - 1] = max[i];
					distance[i - 1] = distance[i];
					theta[i - 1] = theta[i];
				}//shift values in arrays
				max[number_of_lines - 1] = HoughSpace.data[row * HoughSpace.w + col];	// max vote
				distance[number_of_lines - 1] = row;									// perpendecular distance from origin is row index
				theta[number_of_lines - 1] = col;										// theta value is col index of hough space
			}//---------- max vote found and indexed in arrays----
			for (int c = 1; c < number_of_lines - 2; c++)
				if (HoughSpace.data[row * HoughSpace.w + col] > max[number_of_lines - (c + 1)] &&
					HoughSpace.data[row * HoughSpace.w + col] < max[number_of_lines - c]) {
					for (int i = (c + 1); i < number_of_lines; i++) {
						max[i - (c + 1)] = max[i - c];
						distance[i - (c + 1)] = distance[i - c];
						theta[i - (c + 1)] = theta[i - c];
					}//shift values in arrays
					max[number_of_lines - (c + 1)] = HoughSpace.data[row * HoughSpace.w + col];	// max vote
					distance[number_of_lines - (c + 1)] = row;									// perpendecular distance value of max vote
					theta[number_of_lines - (c + 1)] = col;										// theta value of max vote
				}// Check all array if max val grater than any indexif so put it in array
		}


	//--------Distance based elimination for similar lines in houghspace-----------
	for (int i = 0; i < number_of_lines; i++)
		for (int j = i + 1; j < number_of_lines; j++)
			if (abs(distance[j] - distance[i]) <= distQuantize &&
				abs(theta[j] - theta[i]) <= thetaQuantize) {
				if (max[i] >= max[j])
				{
					distance[j] = -1;
					theta[j] = -1;
				}
				else {
					distance[i] = -1;
					theta[i] = -1;
				}
			}
	//--------Paralellism based elimination in lines------
	int* hist = new int[180];
	for (int i = 0; i < 180; i++)
		hist[i] = 0;
	for (int i = 0; i < number_of_lines; i++)
		hist[theta[i]]++;
	int tempmax = 0;
	int maxangle = 0;
	for (int i = 0; i < 180; i++)
		if (hist[i] > tempmax) {
			tempmax = hist[i];
			maxangle = i;
		}
	int tempmax2 = 0;
	int maxangle2 = 0;
	if (maxangle < 90)
		maxangle2 = maxangle + 90;
	else
		maxangle2 = maxangle - 90;
	int limit = 7; // değişken olmalı
	for (int i = 0; i < number_of_lines; i++)
		if (theta[i] != -1) {

			if (!((abs(theta[i] - maxangle) <= limit) ||
				(abs(theta[i] - maxangle2) <= limit) ||
				(theta[i] - limit <= 0 && 0 <= theta[i] + limit) ||
				(theta[i] - limit <= 180 && 180 <= theta[i] + limit)))
			{
				distance[i] = -1;
				theta[i] = -1;
			}
		}
	// Copying line in line struct
	int linecount = 0;
	for (int i = 0; i < number_of_lines; i++)
		if (distance[i] != -1 && theta[i] != -1)
			linecount++;

	Line = new line[linecount];
	numberofline = linecount;

	int k = 0;
	for (int i = 0; i < number_of_lines; i++)
	{
		if (distance[i] != -1 && theta[i] != -1)
		{
			Line[k].max = max[i];
			Line[k].distance = distance[i];
			Line[k].theta = theta[i];

			k++;
		}
	}
	//----------Drawing lines on image---
	int psw, bufpos;
	psw = im.w * 3;
	for (int i = 0; i < linecount; i++)
		for (int row = 3; row < im.h - 3; row++)
			for (int col = 3; col < im.w - 3; col++)
				if (Line[i].distance - HoughSpace.h / 2 == int(col * cos(Line[i].theta * PI / 180) + row * sin(Line[i].theta * PI / 180))) {
					bufpos = (row + 3) * psw + (col + 3) * im.c; // With smoothing kernel, gradient kernel, nonmax kernel, first and last 3 pixels are removed
																 // diff between binary image and rgb image is 3 pixel (row + 3 col+3)						
					im.data[bufpos] = 255, im.data[bufpos + 1] = 0, im.data[bufpos + 2] = 0;
				}


	delete[] max;
	delete[] theta;
	delete[] distance;
	//delete hist;
}
void IntersectionPoint(int numofLines, line* line, hough HoughSpace, point*& p, int& pointnumber, image im) {

	int* x = new int[number_of_points];
	int* y = new int[number_of_points];
	int* id = new int[number_of_points];

	for (int i = 0; i < number_of_points; i++)
	{
		x[i] = -1, y[i] = -1, id[i] = -1;
	}
	// Intersection in Lines
	int count = 0;
	for (int i = 0; i < numofLines; i++)
		for (int j = i + 1; j < numofLines; j++)
			if (parametricIntersect((line[i].distance - (HoughSpace.h / 2)), line[i].theta, (line[j].distance - HoughSpace.h / 2), line[j].theta, x[count], y[count]))
			{
				if ((x[count] > 0 && y[count] > 0) && (x[count] < im.w && y[count] < im.h))
				{
					id[count] = line[j].distance - line[j].theta;
					count++;
				}
				else
				{
					x[count] = -1;
					y[count] = -1;
					id[count] = -1;
				}

			} // if line[i] and line[j] have a intersect hold in x,y array;
	// Similar point elimination
	int xQuant = 10;
	int yQuant = 10;
	for (int i = 0; i < number_of_points; i++)
		for (int j = i + 1; j < number_of_points - 1; j++)
			if (abs(x[j] - x[i]) <= xQuant && abs(y[j] - y[i]) <= yQuant) {
				x[i] = -1;
				y[i] = -1;
				id[i] = -1;
			}
	// Copying point in point struct
	int pointcount = 0;
	for (int i = 0; i < number_of_points; i++)
		if (x[i] != -1)
			pointcount++;
	p = new point[pointcount];
	for (int i = 0; i < pointcount; i++) {
		p[i].X = 0;
		p[i].Y = 0;
		p[i].id = 0;
	}
	int k = 0;
	for (int i = 0; i < number_of_points; i++)
		if (x[i] != -1) {
			p[k].X = x[i];
			p[k].Y = y[i];
			p[k].id = id[i];
			k++;
		}
	pointnumber = pointcount;
	//Drawing point on image
	int psw, bufpos;
	psw = im.w * 3;
	for (int i = 0; i < pointcount - 1; i++)
		for (int theta = 0; theta < 360; theta++) {
			int col = 0, row = 0;
			col = p[i].X + 4 * cos(theta * PI / 180);
			row = p[i].Y + 4 * sin(theta * PI / 180);
			bufpos = (row + 3) * psw + (col + 3) * im.c;
			im.data[bufpos] = 0, im.data[bufpos + 1] = 0, im.data[bufpos + 2] = 255;
		}
	/*delete[] x;
	delete[] y;
	delete[] id;*/
}
int parametricIntersect(int r1, int t1, int r2, int t2, int& x, int& y) {
	/*
		x cos θ1 + y sin θ1 = r1
		x cos θ2 + y sin θ2 = r2

		that is AX = b, where

		A = [cos θ1  sin θ1]   b = |r1|   X = |x|
			[cos θ2  sin θ2]       |r2|       |y|

		d=ct1*st2-st1*ct2

		x=(st2*r1-st1*r2)/d)
		y=((-ct2*r1+ct1*r2)/d)
	*/
	float cost1 = cosf(t1 * PI / 180);
	float sint1 = sinf(t1 * PI / 180);
	float cost2 = cosf(t2 * PI / 180);
	float sint2 = sinf(t2 * PI / 180);

	float d = cost1 * sint2 - sint1 * cost2;
	int tempX = 0;
	int tempY = 0;
	if (d != 0)
	{
		tempX = (int)(sint2 * r1 - sint1 * r2) / d;
		tempY = (int)((-cost2) * r1 + cost1 * r2) / d;
		if (tempX > 0 && tempY > 0)
		{
			x = tempX;
			y = tempY;
			return (1);
		}
		else
			return 0;
	}
	else
		return 0;
}
void Compute_Projection_Matrix(int point_count, float* world_points, point* image_points, float* A)
{
	MatrixXf D(point_count * 2, 11);
	VectorXf R(point_count * 2);

	float X, Y, Z, x, y;
	for (int i = 0; i < point_count; i++)
	{
		X = world_points[i * 3];
		Y = world_points[i * 3 + 1];
		Z = world_points[i * 3 + 2];

		x = image_points[i].X;
		y = image_points[i].Y;

		R(i * 2) = x;
		R(i * 2 + 1) = y;

		float current_point_data[22] = { X,Y,Z,1,0,0,0,0, -X * x, -Y * x, -Z * x,
										0,0,0,0,X,Y,Z,1, -X * y, -Y * y, -Z * y };
		for (int j = 0; j < 11; j++)
		{
			D(i * 2, j) = current_point_data[j];
			D(i * 2 + 1, j) = current_point_data[11 + j];
		}

	}
	VectorXf solution(11);
	JacobiSVD<MatrixXf> svd(D, ComputeFullU | ComputeFullV);
	solution = svd.solve(R);

	for (int i = 0; i < 11; i++)
	{
		A[i] = solution(i);
	}
	A[11] = 1;
}
void Reconstruct(int numofPoint, float* imageArray, float* P, float* worldArray) {
	MatrixXf A(2, 3);
	Vector2f B;
	float x1, y1;

	for (int i = 0; i < numofPoint; i++) {
		x1 = imageArray[i * 2];
		y1 = imageArray[i * 2 + 1];

		A << x1 * P[8] - P[0], x1* P[9] - P[1], x1* P[10] - P[2],
			y1* P[8] - P[4], y1* P[9] - P[5], y1* P[10] - P[6];
		B << P[3] - x1 * P[11],
			P[7] - y1 * P[11];

		Vector3f solution;
		JacobiSVD<MatrixXf> svd(A, ComputeFullU | ComputeFullV);
		solution = svd.solve(B);
		worldArray[i * 3 + 0] = solution(0);
		worldArray[i * 3 + 1] = solution(1);
		worldArray[i * 3 + 2] = solution(2);
	}
}
//std::cout << "avarage x dist " << distanceX << "\t average y dist" << distanceY << "\n";

bool autoCalibVerticalImage(point* points, int numofDetectedP, point*& autoDetectPts, image im) {
	int* dist_histX = new int[400];		// Distance values between two points Histogram
	int* dist_histY = new int[400];
	for (int i = 0; i < 400; i++) {
		dist_histX[i] = 0;
		dist_histY[i] = 0;
	}
	// Compare all points and calc distance histogram
	for (int i = 0; i < numofDetectedP; i++)
		for (int j = i + 1; j < numofDetectedP; j++) {
			if (abs(points[i].Y - points[j].Y) != 0)
				dist_histY[abs(points[i].Y - points[j].Y)]++;
			if (abs(points[i].X - points[j].X) != 0)
				dist_histX[abs(points[i].X - points[j].X)]++;
		}
	//Find the avarage distance in x axis and y axis
	int tempmaxX = 0;
	int tempmaxY = 0;
	int distanceX = 0;
	int distanceY = 0;
	for (int i = 3; i < 400; i++) {
		if (dist_histX[i] > tempmaxX) {
			tempmaxX = dist_histX[i];
			distanceX = i;
		}
		if (dist_histY[i] > tempmaxY) {
			tempmaxY = dist_histY[i];
			distanceY = i;
		}
	}

	// Look for neighbors distance of each point if they has 0-0-0, 20-0-0, 40-0-0
	//														 0-20-0, 20-20-0, 40-20-0 serie return true.
	//
	bool isCalib = false;
	int lim = 2;
	for (int i = 0; i < numofDetectedP; i++) {
		bool left = false; bool upleft = false;	bool up = false;
		bool upright = false; bool right = false;

		int left_index = 0;	int upleft_index = 0; int up_index = 0;
		int upright_index = 0;int right_index = 0;
		for (int j = 0; j < numofDetectedP; j++) {
			if (abs((points[i].X - points[j].X) - distanceX) <= lim && abs(points[i].Y - points[j].Y) <= lim) {
				left = true;
				left_index = j;
			}
			if ((abs((points[i].X - distanceX) - points[j].X) <= lim) && (abs((points[i].Y - distanceY) - points[j].Y) <= lim)) {
				upleft = true;
				upleft_index = j;
			}
			if ((abs((points[i].Y - distanceY) - points[j].Y) <= lim) && abs(points[i].X - points[j].X) <= lim) {
				up = true;
				up_index = j;
			}
			if ((abs((points[i].X + distanceX) - points[j].X) <= lim) && abs((points[i].Y - distanceY) - points[j].Y) <= lim) {
				upright = true;
				upright_index = j;
			}
			if ((abs((points[i].X + distanceX) - points[j].X) <= lim) && abs(points[i].Y - points[j].Y) <= lim) {
				right = true;
				right_index = j;
			}
		}// Check with another loop in point array for neighbour

		if (left && upleft && up && upright && right) {
			autoDetectPts[0].X = points[upleft_index].X;
			autoDetectPts[0].Y = points[upleft_index].Y;
			points[upleft_index].id = 3;
			autoDetectPts[1].X = points[up_index].X;
			autoDetectPts[1].Y = points[up_index].Y;
			points[up_index].id = 5;
			autoDetectPts[2].X = points[upright_index].X;
			autoDetectPts[2].Y = points[upright_index].Y;
			points[upright_index].id = 5;
			autoDetectPts[3].X = points[left_index].X;
			autoDetectPts[3].Y = points[left_index].Y;
			points[left_index].id = 5;
			autoDetectPts[4].X = points[i].X;
			autoDetectPts[4].Y = points[i].Y;
			points[i].id = 5;
			autoDetectPts[5].X = points[right_index].X;
			autoDetectPts[5].Y = points[right_index].Y;
			points[right_index].id = 5;
			isCalib = true;
			break;
		}// if all okay, hold the values

	}//First loop in point array

	 //-------- Draw points in the image
	int psw, bufpos;
	psw = im.w * 3;
	if (isCalib == true) {
		for (int i = 0; i < numofDetectedP; i++) {
			if (points[i].id == 5) {// Neighboor points id is 5 green ones{
				for (int theta = 0; theta < 360; theta++) {
					int col = 0, row = 0;
					col = points[i].X + 4 * cos(theta * PI / 180);
					row = points[i].Y + 4 * sin(theta * PI / 180);
					bufpos = (row + 3) * psw + (col + 3) * im.c;
					im.data[bufpos] = 0, im.data[bufpos + 1] = 255, im.data[bufpos + 2] = 0;
				}
			}
			if (points[i].id == 3) {// Center point in image id is 3 red one{
				for (int theta = 0; theta < 360; theta++) {
					int col = 0, row = 0;
					col = points[i].X + 4 * cos(theta * PI / 180);
					row = points[i].Y + 4 * sin(theta * PI / 180);
					bufpos = (row + 3) * psw + (col + 3) * im.c;
					im.data[bufpos] = 255, im.data[bufpos + 1] = 0, im.data[bufpos + 2] = 0;
				}
			}
		}
		return isCalib;
	}
	else return isCalib;

	// 6 nokta 2 e 3 belirle göster.
	// kareler arası kaç cm 
	// calibre et
	// kullanıcıdan (0,0) noktası seç
	// projeksiyon hesaplanan nokta ile (0,0) noktası farkı hesapla
	// kullanıcıdan ratgele nokta al
	// noktanın gerçek koordinatını hesapla göster


	delete[] dist_histX;
	delete[] dist_histY;
}
//void autoCalibImage(point* points, int numofDetectedP, point*& autoDetectPts, image im)
//{
//	int* dist_histX = new int[400];
//	int* dist_histY = new int[400];
//	for (int i = 0; i < 400; i++)
//	{
//		dist_histX[i] = 0;
//		dist_histY[i] = 0;
//	}
//	
//	for (int i = 0; i < numofDetectedP; i++)
//		for (int j = i + 1; j < numofDetectedP - 1; j++)
//		{
//			if (points[i].id >= 47 && points[i].id <= 133 &&
//				points[j].id >= 47 && points[j].id <= 133)
//			{
//				int distY = abs(points[i].Y - points[j].Y);
//				if(distY>1)
//					dist_histY[distY]++;
//
//			}
//			if ((points[i].id >= 0 && points[i].id <= 43) || (points[i].id >= 137 && points[i].id <= 180) &&
//				(points[j].id >= 0 && points[j].id <= 43) || (points[j].id >= 137 && points[j].id <= 180))
//			{
//				int distX = abs(points[i].X - points[j].X);
//				if(distX >1)
//					dist_histX[distX]++;
//			}
//		}
//	
//	int tempmaxX = 0;
//	int tempmaxY = 0;
//	int distanceX = 0;
//	int distanceY = 0;
//	for (int i = 0; i < 400; i++)
//	{
//		if (dist_histX[i] > tempmaxX) {
//			tempmaxX = dist_histX[i];
//			distanceX = i;
//
//		}
//		if (dist_histY[i] > tempmaxY) {
//			tempmaxY = dist_histY[i];
//			distanceY = i;
//		}
//	}
//	std::cout << "avarage x dist " << distanceX << "\t average y dist" << distanceY << "\n";
//
//	delete[] dist_histX;
//	delete[] dist_histY;
//
//}