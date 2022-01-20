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
#include "opencv2/core/utils/logger.hpp"

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
#include <ctime>

cv::Mat translateImg(cv::Mat& img, int offsetx, int offsety);

std::string datetime();

int main() {
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
	AXISMASK axisMask = (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02);

	std::thread t_scan, t_process;
	cv::Point2d initPos;
	cv::Rect2d printROI;

	// Set the output path with and prepend all file names with the time
	outDir.append(datetime() + "_");

	//Load the raster path generated in Matlab
	double rodLength, rodSpacing;
	std::deque<std::vector<double>> path;
	readPath("pathCoords.txt", rodLength, rodSpacing, path);

	// Make raster
	cv::Mat raster, edgeBoundary;
	std::vector<cv::Point> rasterCoords;
	double border = 2;
	makeRaster(rodLength, rodSpacing, border, 3 - 0.04, raster, edgeBoundary, rasterCoords);
	//Raster ras(rodLength, rodSpacing, border, 1 - 0.04);

	initPos = cv::Point2d(45, 15);
	//initPos = cv::Point2d(95, 15);
	printROI = cv::Rect2d(-border, -border, PIX2MM(raster.cols), PIX2MM(raster.rows)) + initPos;

	// Creating the segmets
	makeSegments(rasterCoords, border, segments, initPos);

	//goto LoadData;

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
	
	//printPath(path, initPos, 3, 0.7);
	t_CollectScans(raster, edgeBoundary, printROI);
	goto cleanup;

	t_scan = std::thread{ t_CollectScans, raster, edgeBoundary, printROI };
	t_process = std::thread{ t_GetMatlErrors, raster };

	// Start the print
	printPath(path, initPos, 1, 0/*0.7*/);

	t_scan.join();
	t_process.join();
	

	// Save the data
	//cv::Mat outData(angles.size(), cols, CV_64FC1);
	//writeCSV("errors.csv", outData);

	//showErrors(img, img, segments);
	//readCSV("edges.csv", gblEdges);
	//temp = showRaster(raster, gblEdges, cv::Scalar(0, 0, 255), 10);

	goto cleanup;

LoadData:
	//// DEBUGGING ERROR CALCULATIONS
	//cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	//readCSV("C:/Users/cu5/source/repos/RasterScanning/scan_and_process/Output/2022.01.20-09.50.48_edges.csv", gblEdges);
	//std::vector<cv::Point> centerline;
	//std::vector<cv::Point> lEdgePts, rEdgePts;
	//std::vector<double> errCL, errWD;
	//double targetWidth = .5;

	//while (segmentNumError < segments.size()) {
	//	// wait for the message to be pushed from the scanning thread
	//	// find and smooth the right and left edges
	//	getMatlEdges(segments[segmentNumError].ROI(), gblEdges, lEdgePts, rEdgePts);
	//	segments[segmentNumError].addEdges(lEdgePts, rEdgePts);
	//	// Calculate Errors
	//	centerline = segments[segmentNumError].centerline();
	//	getMatlErrors(centerline, targetWidth, raster.size(), segments[segmentNumError].lEdgePts(), segments[segmentNumError].rEdgePts(), errCL, errWD);
	//	segments[segmentNumError].addErrors(errCL, errWD, centerline);
	//	segmentNumError++;
	//}
	//cv::Mat image;
	//showErrors(raster, image, segments);
	
cleanup:
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

std::string datetime(){
	time_t rawtime = time(NULL);
	struct tm timeinfo;
	char buffer[80];

	localtime_s(&timeinfo, &rawtime);

	strftime(buffer, 80, "%Y.%m.%d-%H.%M.%S", &timeinfo);
	return std::string(buffer);
}