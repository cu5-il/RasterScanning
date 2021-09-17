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
#include "scanner_functions.h"
#include "processing_functions.h"


// This function will print whatever the latest error was
void PrintError();
void A3200Error(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle);

using namespace cv;
const char* window_name1 = "Edges";

void writeCSV(std::string filename, cv::Mat m)
{
	std::ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
	myfile.close();
}

void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == EVENT_LBUTTONDOWN) {
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
}


int main() {
	Coords fbk;
	A3200Handle handle = NULL;
	A3200DataCollectConfigHandle DCCHandle = NULL;
	AXISMASK axisMask = (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02);
	double initPos[3] = { 232.795, 230.623, -54.883 };
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	cv::Mat Data(NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, CV_64F);

	// INITIALIZATION & Data collection
	//========================================================================================================================
	
	//Connecting to the A3200
	std::cout << "Connecting to A3200. Initializing if necessary.\n";
	if (!A3200Connect(&handle)) { PrintError(); /*goto cleanup;*/ }
	// Creating a data collection handle
	if (!A3200DataCollectionConfigCreate(handle, &DCCHandle)) { PrintError(); /*goto cleanup;*/ }
	// Setting up the data collection
	if (!setupDataCollection(handle, DCCHandle)) { PrintError(); /*goto cleanup;*/ }

	// Homing and moving the axes to the start position
	std::cout << "Enabling and homing X, Y, and Z.\n";
	if (!A3200MotionEnable(handle, TASKID_Library, axisMask)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionHomeConditional(handle, TASKID_Library, axisMask)) { PrintError(); /*goto cleanup;*/ } //home asxes if not already done
	std::cout << "Moving axes to a initial position.\n";
	//if (!A3200MotionLinear(handle, TASKID_Library, axisMask, initPos)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionMoveAbs(handle, TASKID_Library, AXISINDEX_00, initPos[0], 10)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionMoveAbs(handle, TASKID_Library, AXISINDEX_01, initPos[1], 10)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionMoveAbs(handle, TASKID_Library, AXISINDEX_02, initPos[2], 10)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionWaitForMotionDone(handle, axisMask, WAITOPTION_InPosition, -1, NULL)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionDisable(handle, TASKID_Library, axisMask)) { PrintError(); /*goto cleanup;*/ }

	// Collecting the data
	std::cout << "Starting data collection\n";
	if (collectData(handle, DCCHandle, &collectedData[0][0])) {
		// Copy the data to a matrix
		//std::memcpy(Data.data, collectedData, NUM_DATA_SIGNALS * NUM_DATA_SAMPLES * sizeof(double));
	}
	else{ PrintError(); /*goto cleanup;*/ }
	
	// Faking the data
	//std::ifstream inFile("rawData.txt");
	//if (inFile.is_open()) {
	//	for (int i = 0; i < NUM_DATA_SIGNALS; i++) {
	//		for (int j = 0; j < NUM_DATA_SAMPLES; j++) {
	//			inFile >> collectedData[i][j];
	//		}
	//	}
	//	inFile.close();
	//}
	//else {
	//	std::cout << "Unable to open data file" << std::endl;;
	//	system("pause");
	//	return(0);
	//}
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
	cv::Point scanStart, scanEnd;
	cv::Mat scanROI;
	cv::Mat scanGray;
	double input;


	getScan(collectedData, &fbk, scan);
	//writeCSV("scan.csv", scan);

	//cv::normalize(scan, scanGray, 0, 255, cv::NORM_MINMAX, CV_8U);
	//cv::namedWindow("first scan", cv::WINDOW_NORMAL);
	//cv::setMouseCallback("first scan", mouse_callback);
	//cv::imshow("first scan", scanGray);


	// fake ROI
	double printCenter[2] = { fbk.x, 230.6+1.9};
	std::vector<double> printROI = { printCenter[0] - 7, printCenter[1] - 7, printCenter[0] + 7, printCenter[1] + 7 }; //IMPORT FROM FILE

	//Finding the part of the scan that is within the ROI
	scan2ROI(scan, fbk, printROI, raster.size(), scanROI, scanStart, scanEnd);
	cv::normalize(scanROI, scanGray, 0, 255, cv::NORM_MINMAX, CV_8U); // converting so it can be displayed as an image
	cvtColor(scanGray, scanGray, COLOR_GRAY2BGR); // making it 3 channel

	// Finding the edges
	double heightThresh = -1;
	cv::Mat locEdges(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat locWin(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	findEdges(dilation_dst, scanStart, scanEnd, scanROI, gblEdges, locEdges, locWin, heightThresh);

	// Placing the scan on the raster to check alignment
	cv::Mat Z_img_tall;
	int height = 40;
	cv::resize(scanGray, Z_img_tall, Size(scanGray.cols, height), INTER_LINEAR);// Stretching the profile

	// Copy the profile to the raster
	Z_img_tall.copyTo(rasterColor(cv::Rect(scanStart, Z_img_tall.size())), rasterMask(cv::Rect(scanStart, Z_img_tall.size())));
	cv::line(rasterColor, scanStart, scanEnd, Scalar(0, 0, 255), 2); // draw line where scan was taken

	cv::namedWindow("Overlay", cv::WINDOW_NORMAL); 
	cv::setMouseCallback("Overlay", mouse_callback);
	cv::imshow("Overlay", rasterColor);
	

	// displaying the edges

	Mat m(scanGray.size(), CV_8UC3, Scalar({ 255, 0, 255, 0 }));
	Mat b(scanGray.size(), CV_8UC3, Scalar({ 255, 0, 0, 0 }));
	m.copyTo(scanGray, locEdges);
	b.copyTo(scanGray, locWin);
	cv::namedWindow("local", cv::WINDOW_NORMAL);
	cv::setMouseCallback("local", mouse_callback);
	cv::imshow("local", scanGray);

	Mat r(raster.size(), CV_8UC3, Scalar({ 0, 0, 255, 0 }));
	r.copyTo(raster, gblEdges);
	cv::namedWindow("global", cv::WINDOW_NORMAL);
	cv::setMouseCallback("global", mouse_callback);
	cv::imshow("global", raster);

	cv::waitKey(1);

	//-----------------------------------------------------------------------------------------------------------------
	// Moving & Scanning

	std::cout << "Start moving and scanning in X direction" << std::endl;

	std::cout << "Enter zero to keep scanning" << std::endl;
	std::cin >> input;
	if (!A3200MotionEnable(handle, TASKID_Library, (AXISMASK)(AXISMASK_00))) { PrintError(); /*goto cleanup;*/ }

	while (input==0){
		// Move 1 pixel in the X direction
		if (!A3200MotionMoveInc(handle, TASKID_Library, AXISINDEX_00, PIX2MM*20, 1)) { PrintError(); /*goto cleanup;*/ }
		if (!A3200MotionWaitForMotionDone(handle, axisMask, WAITOPTION_MoveDone, -1, NULL)) { PrintError(); /*goto cleanup;*/ }

		// Take a new scan


		std::cout << "Enter a target position or enter a negative value to end the program" << std::endl;
		std::cin >> input;
		input = 10;
	}
	if (!A3200MotionDisable(handle, TASKID_Library, (AXISMASK)(AXISMASK_00))) { PrintError(); /*goto cleanup;*/ }

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// scan again and process
	if (collectData(handle, DCCHandle, &collectedData[0][0])) {
	}
	else { PrintError(); /*goto cleanup;*/ }
	getScan(collectedData, &fbk, scan);
	scan2ROI(scan, fbk, printROI, raster.size(), scanROI, scanStart, scanEnd);

	
	findEdges(dilation_dst, scanStart, scanEnd, scanROI, gblEdges, locEdges, locWin, heightThresh);
	cv::normalize(scanROI, scanGray, 0, 255, cv::NORM_MINMAX, CV_8U); // converting so it can be displayed as an image
	cvtColor(scanGray, scanGray, COLOR_GRAY2BGR); // making it 3 channel

	m = cv::Mat(scanGray.size(), CV_8UC3, Scalar({ 255, 0, 255, 0 }));
	b = cv::Mat(scanGray.size(), CV_8UC3, Scalar({ 255, 0, 0, 0 }));
	m.copyTo(scanGray, locEdges);
	b.copyTo(scanGray, locWin);
	//cv::Mat(scanGray.size(), CV_8UC3, Scalar({ 255, 0, 0, 0 })).copyTo(scanGray, locWin);
	cv::namedWindow("local", cv::WINDOW_NORMAL);
	cv::setMouseCallback("local", mouse_callback);
	cv::imshow("local", scanGray);

	r.copyTo(raster, gblEdges);
	cv::namedWindow("global", cv::WINDOW_NORMAL);
	cv::setMouseCallback("global", mouse_callback);
	cv::imshow("global", raster);
	cv::waitKey(0);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	system("pause");
	return 0;
	//-----------------------------------------------------------------------------------------------------------------


	//========================================================================================================================
cleanup:

	if (NULL != handle) {
		printf("Disconnecting from the A3200.\n");
		if (!A3200Disconnect(handle)) { PrintError(); }
	}
	// Freeing the resources used by the data collection configuration
	if (NULL != DCCHandle) {
		if (!A3200DataCollectionConfigFree(DCCHandle)) { PrintError(); }
	}

#ifdef _DEBUG
	system("pause");
#endif
	return 0;
}

//===================================================================
//							FUNCTIONS
//===================================================================

void PrintError() {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	printf("Error : %s\n", data);
}

void A3200Error(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle) {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	printf("Error : %s\n", data);
	system("pause");

	if (NULL != handle) {
		printf("Disconnecting from the A3200.\n");
		if (!A3200Disconnect(handle)) { PrintError(); }
	}
	// Freeing the resources used by the data collection configuration
	if (NULL != DCCHandle) {
		if (!A3200DataCollectionConfigFree(DCCHandle)) { PrintError(); }
	}
}

//{ A3200Error(handle, DCCHandle); return 0; }
