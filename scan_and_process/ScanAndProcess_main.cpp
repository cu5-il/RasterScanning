#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <string>
#include <algorithm>
#include <iterator> 
#include <valarray>

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
#include "makeRaster.h"
#include "gaussianSmooth.h"
#include "edge_functions.h"
#include "A3200_functions.h"
#include "thread_functions.h"


int main() {
	AXISMASK axisMask = (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02);

	std::thread t1;

	// Make raster
	cv::Mat raster, edgeBoundary;
	std::vector<cv::Point> rasterCoords;
	std::vector<cv::Rect> edgeRegions;
	std::vector<std::vector<cv::Point>> centerlines;
	std::vector<cv::Point2d> scanDonePts;
	double border = 1;
	makeRaster(9, 1, border, 1 - 0.04, raster, edgeBoundary, rasterCoords);

	makeSegments(rasterCoords, border, edgeRegions, centerlines, scanDonePts);
	cv::Point2d initPos = cv::Point2d(0, 0);
	cv::Rect2d printROI = cv::Rect2d(-border, -border, PIX2MM(raster.cols), PIX2MM(raster.rows)) + initPos;
	
	// A3200 Setup
	//=======================================
	//Connecting to the A3200
	std::cout << "Connecting to A3200. Initializing if necessary." << std::endl;
	if (!A3200Connect(&handle)) { A3200Error(); }
	// Creating a data collection handle
	if (!A3200DataCollectionConfigCreate(handle, &DCCHandle)) { A3200Error(); }
	// Setting up the data collection
	if (!setupDataCollection(handle, DCCHandle)) { A3200Error(); }
	// Homing and moving the axes to the start position
	std::cout << "Homing axes." << std::endl;
	if (!A3200MotionEnable(handle, TASKID_Library, axisMask)) { A3200Error(); }
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_00 | AXISMASK_01 ))) { A3200Error(); } //home X & Y axes if not already done
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_02))) { A3200Error(); } //home Z axis if not already done
	if (!A3200MotionWaitForMotionDone(handle, axisMask, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); }
	if (!A3200MotionDisable(handle, TASKID_Library, axisMask)) { A3200Error(); }
	//=======================================

	t1 = std::thread{ t_CollectScans, raster, edgeBoundary, printROI };


	t1.join();

	//A3200 Cleanup
	//=======================================
	// Freeing the resources used by the data collection configuration
	if (NULL != DCCHandle) {
		if (!A3200DataCollectionConfigFree(DCCHandle)) { A3200Error(); }
	}
	// Disconnecting from the A3200
	if (NULL != handle) {
		printf("Disconnecting from the A3200.\n");
		if (!A3200Disconnect(handle)) { A3200Error(); }
	}

#ifdef _DEBUG

#endif
	return 0;
}

