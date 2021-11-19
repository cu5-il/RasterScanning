#include <iostream>

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


void t_CollectScans() {
	cv::Mat scan;
	cv::Point scanStart, scanEnd;
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat scanROI;
	double heightThresh = 0.5;
	cv::Mat locEdges(scanROI.size(), CV_8U, cv::Scalar({ 0 })); // size might be pointless
	cv::Mat locWin(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];

	while (true)
	{
		if (collectData(handle, DCCHandle, &collectedData[0][0])) {
			getScan(collectedData, &fbk, scan);

			//Finding the part of the scan that is within the ROI
			scan2ROI(scan, fbk, printROI, raster.size(), scanROI, scanStart, scanEnd);

			if (!scanROI.empty()) { // Check if the scanROI is empty
				// Finding the edges
				findEdges(edgeBoundary, scanStart, scanEnd, scanROI, gblEdges, locEdges, locWin, heightThresh);
			}
			else { std::cout << "first scan outside of the ROI" << std::endl; }

		}
		else { A3200Error(); }
	}


}

void t_GetMatlErrors() {

}