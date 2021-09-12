#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 

#include <opencv2/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "A3200.h"

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
//#include "scanner_functions.h"
#include "processing_functions.h"


// This function will print whatever the latest error was
//void PrintError();
using namespace cv;

const char* window_name1 = "Edges";



void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == EVENT_LBUTTONDOWN) {
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
}


int main() {
	Coords fbk;
	//A3200Handle handle = NULL;
	//A3200DataCollectConfigHandle DCCHandle = NULL;
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	cv::Mat Data(NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, CV_64F);
	//double X_fb, Y_fb, Z_fb, T_fb;

	// INITIALIZATION & Data collection
	//========================================================================================================================
	/*
	//Connecting to the A3200
	std::cout << "Connecting to A3200. Initializing if necessary.\n";
	if (!A3200Connect(&handle)) { PrintError(); goto cleanup; }
	// Creating a data collection handle
	if (!A3200DataCollectionConfigCreate(handle, &DCCHandle)) { PrintError(); goto cleanup; }
	// Setting up the data collection
	if (!setupDataCollection(handle, DCCHandle)) { PrintError(); goto cleanup; }

	// Collecting the data
	std::cout << "Starting data collection\n";
	if (collectData(handle, DCCHandle, &collectedData[0][0])) {
		// Copy the data to a matrix
		std::memcpy(Data.data, collectedData, NUM_DATA_SIGNALS * NUM_DATA_SAMPLES * sizeof(double));
	}
	else{ PrintError(); goto cleanup; }
	*/

	// Faking the data
	std::ifstream inFile("rawData.txt");
	if (inFile.is_open()) {
		for (int i = 0; i < NUM_DATA_SIGNALS; i++) {
			for (int j = 0; j < NUM_DATA_SAMPLES; j++) {
				inFile >> collectedData[i][j];
			}
		}
		inFile.close();
	}
	else {
		std::cout << "Unable to open data file" << std::endl;;
		system("pause");
		return(0);
	}
	//========================================================================================================================

	double xfb, yfb, /*zfb,*/ Tfb;
	cv::Mat profile, Z;
	
	processData(collectedData, &fbk, profile);

	// fake feedback coordinates
	fbk.x = 10;
	fbk.y = 9.8;
	fbk.T = 0;


	// Calculating the X & Y coordinates of the scan
	// --------------------------------------------------------------------------

	std::vector<double> printROI = { -1, -1, 13, 13 }; //IMPORT FROM FILE

	cv::Point profileStart, profileEnd;
	std::vector<Point> profilePts(2);
	cv::Range profileROIRange;
	local2globalScan((int)profile.cols, fbk, printROI, profileStart, profileEnd, profileROIRange);

	// Interpolate scan so it is the same scale as the raster reference image
	//cv::resize(profile.colRange(profileROIRange), profile, Size(RASTER_IMG_SIZE, profile.rows), INTER_LINEAR);
	cv::resize(profile.colRange(profileROIRange), profile, Size(profileEnd.x-profileStart.x, profile.rows), INTER_LINEAR);

	std::cout << "fcn coordinates (PIX) = " << profileStart << " to " << profileEnd << std::endl;

	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
	cv::Mat Zroi = profile;
	cv::Mat Zroi_img;
	cv::normalize(Zroi, Zroi_img, 0, 255, cv::NORM_MINMAX, CV_8U);

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Loading the raster image
	cv::Mat raster, rasterMask;
	raster = cv::imread("RasterMap.bmp", IMREAD_COLOR);
	cv::Mat rasterColor(raster.size(), CV_8UC3, Scalar({ 0, 255, 0, 0 }));
	bitwise_not(raster, rasterMask);
	raster.copyTo(rasterColor, rasterMask);


	// Stretching the profile
	cv::Mat Z_img_tall;
	int height = 40;
	cv::resize(Zroi_img, Z_img_tall, Size(Zroi_img.cols, height), INTER_LINEAR);
	cvtColor(Z_img_tall, Z_img_tall, COLOR_GRAY2BGR);

	// Copy the profile to the raster
	Z_img_tall.copyTo(rasterColor(cv::Rect(profileStart, Z_img_tall.size())), rasterMask(cv::Rect(profileStart, Z_img_tall.size())));


	cv::line(rasterColor, profileStart, profileEnd, Scalar(0, 0, 255), 2); // draw line where scan was taken

	//cv::namedWindow("Overlay", cv::WINDOW_AUTOSIZE); 
	//cv::setMouseCallback("Overlay", mouse_callback);
	//cv::imshow("Overlay", rasterColor); 


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Making the region around the raster path to search for edges
	Mat dilation_dst;
	int dilation_size = 18;// 13 wa sa little too small; 20 seems ok; 23 is max
	Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	dilate(raster, dilation_dst, element);

	//cv::namedWindow("Search Mask", cv::WINDOW_AUTOSIZE);
	//cv::setMouseCallback("Search Mask", mouse_callback);
	//cv::imshow("Search Mask", dilation_dst);

	// Using a line iterator to extract the points along the scan line
	cv::LineIterator Lit(dilation_dst, profileStart, profileEnd);

	std::vector<Point> points;
	points.reserve(Lit.count);
	for (int i = 0; i < Lit.count; ++i, ++Lit) {
		points.push_back(Lit.pos());
	}
	//std::cout <<"line coords "<< points << std::endl;

	// Alternative line iteration
	//https://docs.opencv.org/4.5.1/dc/dd2/classcv_1_1LineIterator.html
	LineIterator it(dilation_dst, profileStart, profileEnd, 8);
	LineIterator it2 = it;
	std::vector<uchar> buf(it.count);
	std::vector<Point> points2;
	points2.reserve(it2.count);
	uchar lastVal = 0;
	uchar curVal = 0;
	// find the edges of the dialeted raster 
	for (int i = 0; i < it.count; i++, ++it) {
		//	buf[i] = *(const uchar*)*it;
		curVal = *(const uchar*)*it;
		if ((curVal == 255) && (lastVal == 0) && (i != 0)) { // find rising edges
			points2.push_back(it.pos());
		}
		if ((curVal == 0) && (lastVal == 255) && (i != 0)) { // find falling edges
			points2.push_back(it.pos());
		}
		lastVal = curVal;
	}
	if ((points2.size() % 2) != 0) {
		std::cout << "ERROR: odd number of edges" << std::endl;
		return 0;
	}
	std::cout << "Search from:" << std::endl;
	//-------------------------------------------------------------------------------------------------------------
	// Search within the edges of the dialated raster for the actual edges
	cv::Mat searchROI;
	cv::Mat ROIedges;
	std::vector<Point> ROIedgeIdx;
	cvtColor(Zroi_img, Zroi_img, COLOR_GRAY2BGR); //convert global image to color
	for (int i = 0; i < points2.size(); i = i + 2) {

		std::cout << "-------------" << std::endl;
		std::cout << points2[i].x << " to " << points2[i + 1].x << std::endl;
		searchROI = Zroi(cv::Range::all(), cv::Range(points2[i].x, points2[i + 1].x)); // isolate single peak
		cv::normalize(searchROI, searchROI, 0, 255, cv::NORM_MINMAX, CV_8U);
		cv::Canny(searchROI, ROIedges, 10, 20, 7);

		findNonZero(ROIedges, ROIedgeIdx);

		cv::Vec3b edgeColor;
		if (ROIedgeIdx.size() == 2) { //verify that only two edges were found
			edgeColor = Vec3b(0, 0, 255);
		}
		else {
			//edgeColor = Vec3b(0, 128, 255); //orange
			edgeColor = Vec3b(255, 0, 188); //purple
		}

		//mark search regions on global image
		Zroi_img.at<Vec3b>(Point(points2[i].x, 0)) = Vec3b(255, 0, 0);
		Zroi_img.at<Vec3b>(Point(points2[i + 1].x, 0)) = Vec3b(255, 0, 0);

		for (int j = 0; j < ROIedgeIdx.size(); j++) {
			Zroi_img.at<Vec3b>(Point(ROIedgeIdx[j].x + points2[i].x, 0)) = edgeColor;
		}
		std::cout << "Num edges = " << ROIedgeIdx.size() << std::endl;
		std::cout << ROIedgeIdx << std::endl;

		//cv::namedWindow("ROI", cv::WINDOW_NORMAL);
		//cv::setMouseCallback("ROI", mouse_callback);
		//cv::imshow("ROI", searchROI);
		//cv::namedWindow("ROI edges", cv::WINDOW_NORMAL);
		//cv::setMouseCallback("ROI edges", mouse_callback);
		//cv::imshow("ROI edges", ROIedges);
		//waitKey(1);
	}
	std::cout << "-------------" << std::endl;

	cv::namedWindow("Global ROI", cv::WINDOW_NORMAL);
	cv::setMouseCallback("Global ROI", mouse_callback);
	cv::imshow("Global ROI", Zroi_img);



	cv::waitKey(0);
	return 0;
	//-----------------------------------------------------------------------------------------------------------------




		// Making a image to store all the global information
	Mat globalImg(raster.size(), CV_8UC3, Scalar({ 0, 0, 0, 0 }));

	Z_img_tall.copyTo(globalImg(cv::Rect(profileStart, Z_img_tall.size())), dilation_dst(cv::Rect(profileStart, Z_img_tall.size())));

	Mat globalEdge;
	cv::Canny(globalImg, globalEdge, 10, 30, 3);
	Mat red(raster.size(), CV_8UC3, Scalar({ 0, 0, 255, 0 }));
	red.copyTo(globalImg, globalEdge);


	//cv::namedWindow("global", cv::WINDOW_AUTOSIZE); // Create a window for display.
	//cv::setMouseCallback("global", mouse_callback);
	//cv::imshow("global", globalImg); // Show our image inside it.

	cv::waitKey(0); // Wait for a keystroke in the window

	//========================================================================================================================
//cleanup:
//
//	if (NULL != handle) {
//		printf("Disconnecting from the A3200.\n");
//		if (!A3200Disconnect(handle)) { PrintError(); }
//	}
//	// Freeing the resources used by the data collection configuration
//	if (NULL != DCCHandle) {
//		if (!A3200DataCollectionConfigFree(DCCHandle)) { PrintError(); }
//	}

#ifdef _DEBUG
	system("pause");
#endif
	return 0;
}

//===================================================================
//							FUNCTIONS
//===================================================================

//void PrintError() {
//	CHAR data[1024];
//	A3200GetLastErrorString(data, 1024);
//	printf("Error : %s\n", data);
//}
