#include <iostream>
#include <algorithm>

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


void t_CollectScans(const cv::Mat raster, const cv::Mat edgeBoundary, cv::Rect2d printROI) {
	cv::Mat scan;
	cv::Point scanStart, scanEnd;
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat scanROI;
	double heightThresh = 0.5;
	cv::Mat locEdges(scanROI.size(), CV_8U, cv::Scalar({ 0 })); // size might be pointless
	cv::Mat locWin(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	Coords scanPosFbk;

	while (true){
		if (collectData(handle, DCCHandle, &collectedData[0][0])) {
			// Trigger the scanner and collect the scanner data
			getScan(collectedData, &scanPosFbk, scan);
			//Find the part of the scan that is within the ROI of the print
			scan2ROI(scan, scanPosFbk, printROI, raster.size(), scanROI, scanStart, scanEnd);
			// Check if the scanROI is empty
			if (!scanROI.empty()) { 
				// Finding the edges
				findEdges(edgeBoundary, scanStart, scanEnd, scanROI, gblEdges, locEdges, locWin, heightThresh);
			}
			else { std::cout << "scan outside of the ROI" << std::endl; }
		}
		else { A3200Error(); }

		if (positionFlag) {
			// push the edges to the error calculating thread
			q_scannedEdges.push(gblEdges);
			segmentNum++; 
		}
	}


}

void t_GetMatlErrors(const cv::Mat raster, std::vector<cv::Point> rasterCoords) {
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	std::vector<cv::Rect> edgeRegions;
	std::vector<std::vector<cv::Point>> centerlines;
	std::vector<cv::Point> scanDonePts;
	std::vector<cv::Point> lEdgePts, rEdgePts;
	std::vector<cv::Point> newCentLine, newlEdge, newrEdge;
	std::vector<std::vector<double>> errCL, errWD;
	double targetWidth = 0;
	int regionNum = 0;
	
	// Making the edge search regoins
	double border = 1;
	makeSegments(rasterCoords, border, edgeRegions, centerlines, scanDonePts);
	
	while (!doneScanning){
		// wait for the edges to be push from the scanning thread
		q_scannedEdges.wait_and_pop(gblEdges);

		// find and smooth the right and left edges
		getMatlEdges(edgeRegions[regionNum], gblEdges, lEdgePts, rEdgePts);

		// Calculate Errors
		getMatlErrors(centerlines[regionNum], targetWidth, raster.size(), lEdgePts, rEdgePts, errCL, errWD);


	}
}