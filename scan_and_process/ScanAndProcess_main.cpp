#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <string>
#include <algorithm>
#include <iterator> 
#include <valarray>
#include <deque>

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
#include "csvMat.h"
#include "motion.h"
#include "raster.h"
#include "matlab.h"

cv::Mat translateImg(cv::Mat& img, int offsetx, int offsety);

int main() {
	AXISMASK axisMask = (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02);

	std::thread t_scan, t_process;
	cv::Point2d initPos;
	cv::Rect2d printROI;

	//Load the raster path generated in Matlab
	double rodLength, rodSpacing;
	std::deque<std::vector<double>> path;
	readPath("pathCoords.txt", rodLength, rodSpacing, path);

	// Make raster
	cv::Mat raster, edgeBoundary;
	std::vector<cv::Point> rasterCoords;
	double border = 1;
	makeRaster(rodLength, rodSpacing, border, 1 - 0.04, raster, edgeBoundary, rasterCoords);
	//Raster ras(rodLength, rodSpacing, border, 1 - 0.04);

	goto skipsetup;

	// A3200 Setup
	//=======================================
	//Connecting to the A3200
	std::cout << "Connecting to A3200. Initializing if necessary." << std::endl;
	if (!A3200Connect(&handle)) { A3200Error(); }
	// Creating a data collection handle
	if (!A3200DataCollectionConfigCreate(handle, &DCCHandle)) { A3200Error(); }
	// Setting up the data collection
	if (!setupDataCollection(handle, DCCHandle)) { A3200Error(); }
	// Initializing the extruder
	extruder.initialize(handle);
	// Homing and moving the axes to the start position
	std::cout << "Homing axes." << std::endl;
	if (!A3200MotionEnable(handle, TASKID_Library, axisMask)) { A3200Error(); }
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_00 | AXISMASK_01 ))) { A3200Error(); } //home X & Y axes if not already done
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_02))) { A3200Error(); } //home Z axis if not already done
	if (!A3200MotionWaitForMotionDone(handle, axisMask, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); }
	if (!A3200MotionDisable(handle, TASKID_Library, axisMask)) { A3200Error(); }
	//=======================================
skipsetup:
	initPos = cv::Point2d(0, 0);
	printROI = cv::Rect2d(-border, -border, PIX2MM(raster.cols), PIX2MM(raster.rows)) + initPos;

	// Creating the segmets
	makeSegments(rasterCoords, border, segments);

	printPath(path, initPos, 1, 0.5);




	goto skipThreading;
	//// LOAD DATA TO TEST MULTITHREADING
	//cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	//readCSV("C:/Users/cu5/Box/Research/Code/Image Processing/Edge Data/gbledges_raster.csv", gblEdges);
	//gblEdges.convertTo(gblEdges, CV_8UC1);
	////HACK: Shifting glbEdges by a few pixels to the leftso it aligns better
	//translateImg(gblEdges, -2, 0);
	//cv::Mat img = showRaster(raster, gblEdges, cv::Scalar(155, 155, 155));

	t_scan = std::thread{ t_CollectScans, raster, edgeBoundary, printROI };
	//t_scan = std::thread{ t_CollectScans, gblEdges, edgeBoundary, printROI }; // HACK: passed edges in through raster input
	t_process = std::thread{ t_GetMatlErrors, raster };

	t_scan.join();
	t_process.join();
	std::cout << "All segments have been processed" << std::endl;

	//showErrors(img, img, segments);
skipThreading:

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

cv::Mat translateImg(cv::Mat& img, int offsetx, int offsety) {
	cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
	cv::warpAffine(img, img, trans_mat, img.size());
	return img;
}