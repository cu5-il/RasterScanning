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

	double initPos[3] = { 221 - RASTER_IMG_WIDTH / 2.0, 229.7, -54.35 };
	// fake ROI
	double printCenter[2] = { 221.0, 229.7};//{ 234.75, 230.25 };
	std::vector<double> printROI = { printCenter[0] - RASTER_IMG_WIDTH / 2.0, printCenter[1] - RASTER_IMG_WIDTH / 2.0, printCenter[0] + RASTER_IMG_WIDTH / 2.0, printCenter[1] + RASTER_IMG_WIDTH / 2.0 }; //IMPORT FROM FILE

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
	
	//========================================================================================================================

	cv::Mat scan;
	cv::Point scanStart, scanEnd;
	cv::Mat scanROI;
	double input;

	getScan(collectedData, &fbk, scan);
	//writeCSV("scan.csv", scan);

	
	//Finding the part of the scan that is within the ROI
	scan2ROI(scan, fbk, printROI, raster.size(), scanROI, scanStart, scanEnd);

	// Finding the edges
	double heightThresh = 0.5;
	cv::Mat locEdges(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat locWin(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	findEdges(edgeBoundary, scanStart, scanEnd, scanROI, gblEdges, locEdges, locWin, heightThresh);

	// Displaying images
	cv::Mat img1 = showOverlay(raster, scanROI, scanStart, scanEnd);
	cv::Mat img2 = showScan(scanROI, locEdges, locWin);
	cv::Mat img3 = showRaster(raster, gblEdges);
	cv::Mat superImg(cv::Size(img1.cols + img3.cols, img2.rows + img3.rows), CV_8UC3);
	img3.copyTo(superImg(cv::Rect(cv::Point(0, 0), img3.size())));
	img2.copyTo(superImg(cv::Rect(cv::Point(0, img3.rows), img2.size())));
	img1.copyTo(superImg(cv::Rect(cv::Point(img3.cols,0), img1.size())));

	//~~~~~~~~~~~~~
	// setting up movie
	const char* movName = "myVid.avi";
	cv::Size S = superImg.size();
	cv::VideoWriter outputVideo;  // Open the output
	outputVideo.open(movName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 1, S, true);  //30 for 30 fps
	if (!outputVideo.isOpened()) {
		std::cout << "Could not open the output video for write: " << std::endl;
		return -1;
	}
	//~~~~~~~~~~~~~

	//-----------------------------------------------------------------------------------------------------------------
	// Moving & Scanning

	std::cout << "Start moving and scanning in X direction" << std::endl;
	if (!A3200MotionEnable(handle, TASKID_Library, (AXISMASK)(AXISMASK_00))) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionMoveInc(handle, TASKID_Library, AXISINDEX_00, RASTER_IMG_WIDTH, 0.2)) { PrintError(); /*goto cleanup;*/ }

	std::vector<cv::Mat> frames;
	int scanCount = 0;
	while (fbk.x <(printROI[2]-0.25)){ // Scan until outside of ROI
		// Move 1 pixel in the X direction
		if (collectData(handle, DCCHandle, &collectedData[0][0])) {
			getScan(collectedData, &fbk, scan);
			scan2ROI(scan, fbk, printROI, raster.size(), scanROI, scanStart, scanEnd);
			findEdges(edgeBoundary, scanStart, scanEnd, scanROI, gblEdges, locEdges, locWin, heightThresh);
			// Displaying images
			//showOverlay(raster, scanROI, scanStart, scanEnd);
			//showScan(scanROI, locEdges, locWin);
			//showRaster(raster, gblEdges);
			img1 = showOverlay(raster, scanROI, scanStart, scanEnd);
			img2 = showScan(scanROI, locEdges, locWin);
			img3 = showRaster(raster, gblEdges);
			img3.copyTo(superImg(cv::Rect(cv::Point(0, 0), img3.size())));
			img2.copyTo(superImg(cv::Rect(cv::Point(0, img3.rows), img2.size())));
			img1.copyTo(superImg(cv::Rect(cv::Point(img3.cols, 0), img1.size())));
			outputVideo << superImg;
			//frames.push_back(superImg);
			scanCount++;
		}
		else { PrintError(); /*goto cleanup;*/ }
	}
	// wait for all motion to complete and then disable the axes
	if (!A3200MotionWaitForMotionDone(handle, axisMask, WAITOPTION_MoveDone, -1, NULL)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionDisable(handle, TASKID_Library, (AXISMASK)(AXISMASK_00))) { PrintError(); /*goto cleanup;*/ }

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Displaying images
	//showOverlay(raster, scanROI, scanStart, scanEnd);
	//showScan(scanROI, locEdges, locWin);
	//showRaster(raster, gblEdges);
	std::cout << "All movement done. " <<scanCount <<" scans taken."<< std::endl;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// make movie
	//cv::namedWindow("frame1", cv::WINDOW_NORMAL);
	//cv::imshow("frame1", frames[0]);
	//cv::namedWindow("frame80", cv::WINDOW_NORMAL);
	//cv::imshow("frame80", frames[80]);

//const char* movName = "myVid.avi";
//cv::Size S = superImg.size();
//cv::VideoWriter outputVideo;  // Open the output
//outputVideo.open(movName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 1, S, true);  //30 for 30 fps
//if (!outputVideo.isOpened()) {
//	std::cout << "Could not open the output video for write: " << std::endl;
//	return -1;
//}

	//for (int i = 0; i < frames.size(); i++) {
	//	outputVideo << frames[i];
	//	//outputVideo.write(frames[i]);
	//}
	outputVideo.release();
	std::cout << "Finished writing" << std::endl;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	cv::waitKey(0);
	//system("pause");

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
