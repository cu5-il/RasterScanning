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

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Loading the raster image
	cv::Mat raster, rasterMask;
	raster = cv::imread("RasterMap.bmp", IMREAD_COLOR);
	cv::Mat rasterColor(raster.size(), CV_8UC3, Scalar({ 0, 255, 0, 0 }));
	bitwise_not(raster, rasterMask);
	raster.copyTo(rasterColor, rasterMask);

	// Making the region around the raster path to search for edges
	Mat dilation_dst;
	cv::Mat edgeSearchROI;
	int dilation_size = 18;// 13 wa sa little too small; 20 seems ok; 23 is max
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
	dilate(raster, dilation_dst, element);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	cv::Mat scan;
	
	processData(collectedData, &fbk, scan);

	// fake feedback coordinates
	fbk.x = 10;
	fbk.y = 9.8;
	fbk.T = 30;
	
	// fake ROI
	std::vector<double> printROI = { -1, -1, 13, 13 }; //IMPORT FROM FILE

	// Calculating the X & Y coordinates of the scan
	// --------------------------------------------------------------------------

	
	//Finding the part of the scan that is within the ROI
	cv::Point profileStart, profileEnd;
	cv::Mat scanROI;
	scan2ROI(scan, fbk, printROI, raster.size(), scanROI, profileStart, profileEnd);

	LineIterator lit(profileStart, profileEnd, 8);
	LineIterator lit2(Size(700,700),profileStart, profileEnd, 8);
	std::cout << "Length = " << lit2.count << std::endl;
	std::cout << "Length = " << lit.count << std::endl;
	std::cout << "Length = " << norm(profileStart-profileEnd) << std::endl;
	std::cout << "Length = " << sqrt(pow(profileStart.x - profileEnd.x, 2) + pow(profileStart.y - profileEnd.y, 2)) << std::endl;
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

	cv::Mat scanGray;
	cv::normalize(scanROI, scanGray, 0, 255, cv::NORM_MINMAX, CV_8U);


	// Finding the edges
	double heightThresh = 2.2;
	cv::Mat locEdges(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	findEdges(dilation_dst, profileStart, profileEnd, scanROI, gblEdges, locEdges, heightThresh);

	cv::namedWindow("local edges", cv::WINDOW_NORMAL);
	cv::setMouseCallback("local edges", mouse_callback);
	cv::imshow("local edges", locEdges);

	// displaying the edges
	Mat red2(scanGray.size(), CV_8UC3, Scalar({ 255, 0, 255, 0 }));
	cvtColor(scanGray, scanGray, COLOR_GRAY2BGR);
	red2.copyTo(scanGray, locEdges);
	cv::namedWindow("local", cv::WINDOW_NORMAL);
	cv::setMouseCallback("local", mouse_callback);
	cv::imshow("local", scanGray);

	Mat red3(raster.size(), CV_8UC3, Scalar({ 0, 0, 255, 0 }));
	red3.copyTo(raster, gblEdges);
	cv::namedWindow("global", cv::WINDOW_NORMAL);
	cv::setMouseCallback("global", mouse_callback);
	cv::imshow("global", raster);

	cv::waitKey(0);
	return 0;
	//-----------------------------------------------------------------------------------------------------------------


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
