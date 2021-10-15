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
#include "makeRaster.h"


// This function will print whatever the latest error was
//void PrintError();
using namespace cv;

const char* window_name1 = "Edges";

void writeCSV(std::string filename, cv::Mat m)
{
	std::ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
	myfile.close();
}

void readCSV(std::string filename, cv::Mat& m)
{
	std::ifstream inFile(filename.c_str());
	std::string strval;
	std::vector<double> vec;
	double value;
	//myfile.open();
	if (inFile.is_open()) {
		while (std::getline(inFile, strval,',')) {
			value = std::stod(strval);
			vec.push_back(value);
		}
	}
	else {
		std::cout << "Unable to open data file" << std::endl;;
		system("pause");
		return;
	}
	//std::memcpy(m.data, vec.data(), vec.size() * sizeof(double));
	m = cv::Mat (1, vec.size(), CV_64FC1, vec.data());
	m = m.clone();
	return;
}

void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == EVENT_LBUTTONDOWN) {
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
}


int main() {
	cv::Mat raster1, edgeBoundary1;
	std::vector<cv::Point> rasterCoords;
	makeRaster(12, 1, 1, 1-0.04, raster1, edgeBoundary1, rasterCoords);
	//========================================================================================================================

	Coords fbk;
	//A3200Handle handle = NULL;
	//A3200DataCollectConfigHandle DCCHandle = NULL;
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	cv::Mat Data(NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, CV_64F);

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

	std::vector<cv::Point> coords;
	cv::cvtColor(raster, raster, COLOR_BGR2GRAY);
	cv::flip(raster, raster, -1);
	cv::findNonZero(raster, coords);
	

	// Making the region around the raster path to search for edges
	Mat dilation_dst;
	cv::Mat edgeSearchROI;
	int dilation_size = 18;// 13 was a little too small; 20 seems ok; 23 is max
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
	dilate(raster, dilation_dst, element);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	cv::Mat scan;
	
	getScan(collectedData, &fbk, scan);

	writeCSV("output.csv", scan);

	// fake feedback coordinates
	fbk.x = 10;
	fbk.y = 9.8;
	fbk.T = 0;
	
	// fake ROI
	std::vector<double> printROI = { -1, -1, 13, 13 }; //IMPORT FROM FILE

	////injecting in the other scan
	//readCSV("scan2.csv", scan);
	//fbk.y = 5.2; // CHANGED HEIGHT THRESH TO NEGATIVE

	//Finding the part of the scan that is within the ROI
	cv::Point scanStart, scanEnd;
	cv::Mat scanROI;
	scan2ROI(scan, fbk, printROI, raster.size(), scanROI, scanStart, scanEnd);

	// Finding the edges
	double heightThresh = -2.2;
	heightThresh = 1;
	cv::Mat locEdges(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	//findEdges(raster, scanStart, scanEnd, scanROI, gblEdges, locEdges, heightThresh);
	findEdges(edgeBoundary1, scanStart, scanEnd, scanROI, gblEdges, locEdges, heightThresh);

	//// template matching
	//cv::Mat templ = scanROI(cv::Range::all(), cv::Range(81, 118));
	//templ.convertTo(templ, CV_32F);
	//scanROI.convertTo(scanROI, CV_32F);
	//Mat result;
	//matchTemplate(scanROI, templ, result, TM_CCOEFF);
	//writeCSV("output.csv", scanROI);
	//cv::namedWindow("result", cv::WINDOW_NORMAL);
	//cv::imshow("result", result);
	//std::cout << "result = " << result << std::endl;
	//writeCSV("output.csv", result);
	// displaying the edges

	cv::Mat scanGray;
	cv::normalize(scanROI, scanGray, 0, 255, cv::NORM_MINMAX, CV_8U);

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

