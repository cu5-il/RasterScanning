#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <string>

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
#include "display_functions.h"


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


int main() {
	Coords fbk;
	A3200Handle handle = NULL;
	A3200DataCollectConfigHandle DCCHandle = NULL;
	AXISMASK axisMask = (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02);
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	cv::Mat Data(NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, CV_64F);

	//double initPos[3] = { 221 - RASTER_IMG_WIDTH / 2.0, 229.5, -54.45 };
	//double printCenter[2] = { 221.0, 229.7};//{ 234.75, 230.25 };
	double initPos[3] = { 220.5 - RASTER_IMG_WIDTH / 2.0, 249.0, -54.45 };
	// fake ROI
	double printCenter[2] = { 220.0, 249.0};//{ 234.75, 230.25 };
	std::vector<double> printROI = { printCenter[0] - RASTER_IMG_WIDTH / 2.0,
		printCenter[1] - RASTER_IMG_WIDTH / 2.0,
		printCenter[0] + RASTER_IMG_WIDTH / 2.0,
		printCenter[1] + RASTER_IMG_WIDTH / 2.0 
	}; //IMPORT FROM FILE

	// INITIALIZATION & Data collection
	//========================================================================================================================
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Loading the raster image
	cv::Mat raster;
	raster = cv::imread("RasterMap9.bmp", IMREAD_COLOR);

	// Making the region around the raster path to search for edges
	cv::Mat edgeBoundary;
	int dilation_size = 20;// was 18 when using 14x14 raster image; 20 seems ok; 23 is max
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));
	cv::dilate(raster, edgeBoundary, element);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
	std::cout << "Moving axes to initial position.\n";
	if (!A3200MotionMoveAbs(handle, TASKID_Library, AXISINDEX_00, initPos[0], 10)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionMoveAbs(handle, TASKID_Library, AXISINDEX_01, initPos[1], 10)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionMoveAbs(handle, TASKID_Library, AXISINDEX_02, initPos[2], 10)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionWaitForMotionDone(handle, axisMask, WAITOPTION_InPosition, -1, NULL)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionDisable(handle, TASKID_Library, axisMask)) { PrintError(); /*goto cleanup;*/ }


	//========================================================================================================================

	cv::Mat scan;
	cv::Point scanStart, scanEnd;
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat scanROI;
	double heightThresh = 0.5;
	cv::Mat locEdges(scanROI.size(), CV_8U, cv::Scalar({ 0 })); // size might be pointless
	cv::Mat locWin(scanROI.size(), CV_8U, cv::Scalar({ 0 }));


	std::cout << "Starting data collection\n";
	if (collectData(handle, DCCHandle, &collectedData[0][0])) {
		getScan(collectedData, &fbk, scan);

		//Finding the part of the scan that is within the ROI
		scan2ROI(scan, fbk, printROI, raster.size(), scanROI, scanStart, scanEnd);

		if (!scanROI.empty()) { // Check if the scanROI is empty
			// Finding the edges
			findEdges(edgeBoundary, scanStart, scanEnd, scanROI, gblEdges, locEdges, locWin, heightThresh);
			// Displaying images
			showOverlay(raster, scanROI, scanStart, scanEnd, true);
			showScan(scanROI, locEdges, locWin, true);
			showRaster(raster, gblEdges, true);
		}
		else { std::cout << "first scan outside of the ROI" << std::endl; }

	}
	else { PrintError(); /*goto cleanup;*/ }



	////~~~~~~~~~~~~~
	//// setting up movie
	//const char* movName = "myVid.avi";
	//cv::Size S = cv::Size(2 * raster.cols, raster.rows + 100 /*scan_img.rows*/);
	//cv::VideoWriter outputVideo;  // Open the output
	//outputVideo.open(movName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 1, S, true);  //1 for 1 fps
	////outputVideo.open(movName, cv::VideoWriter::fourcc('I', '4', '2', '0'), 1, S, true);  //1 for 1 fps - uncompressed video
	//if (!outputVideo.isOpened()) {
	//	std::cout << "Could not open the output video for write: " << std::endl;
	//	return -1;
	//}
	////~~~~~~~~~~~~~

	//-----------------------------------------------------------------------------------------------------------------
	// Moving & Scanning

	std::cout << "Start moving and scanning in X direction the width of the raster" << std::endl;
	if (!A3200MotionEnable(handle, TASKID_Library, (AXISMASK)(AXISMASK_00))) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionMoveInc(handle, TASKID_Library, AXISINDEX_00, RASTER_IMG_WIDTH, 0.2)) { PrintError(); /*goto cleanup;*/ }

	while (fbk.x < (printROI[2] - 0.25)) { // Scan until outside of ROI

		if (collectData(handle, DCCHandle, &collectedData[0][0])) {
			getScan(collectedData, &fbk, scan);
			if (!scanROI.empty()) { // Check if the scanROI is empty
				scan2ROI(scan, fbk, printROI, raster.size(), scanROI, scanStart, scanEnd);
				findEdges(edgeBoundary, scanStart, scanEnd, scanROI, gblEdges, locEdges, locWin, heightThresh);
				// Displaying images
				showOverlay(raster, scanROI, scanStart, scanEnd, true);
				showScan(scanROI, locEdges, locWin, true);
				showRaster(raster, gblEdges, true);

				//outputVideo << showAll(raster, scanROI, scanStart, scanEnd, locEdges, locWin, gblEdges, true);
			}
		}
		else { PrintError(); /*goto cleanup;*/ }
	}
	// wait for all motion to complete and then disable the axes
	if (!A3200MotionWaitForMotionDone(handle, axisMask, WAITOPTION_MoveDone, -1, NULL)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionDisable(handle, TASKID_Library, (AXISMASK)(AXISMASK_00))) { PrintError(); /*goto cleanup;*/ }
	
	writeCSV("gbledges.csv", gblEdges);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Displaying images
	showOverlay(raster, scanROI, scanStart, scanEnd, true);
	showScan(scanROI, locEdges, locWin, true);
	showRaster(raster, gblEdges, true);
	std::cout << "All movement done. " << std::endl;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


	//outputVideo.release();
	//std::cout << "Finished writing video" << std::endl;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	cv::waitKey(0);


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
