#include <iostream>
#include <algorithm>

#include "A3200.h"
#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "scanner_functions.h"
#include "processing_functions.h"
#include "display_functions.h"
#include "edge_functions.h"
#include "A3200_functions.h"
#include "csvMat.h"
#include "raster.h"


void t_CollectScans(Raster raster) {
	cv::Mat scan(1, NUM_DATA_SAMPLES, CV_64F);
	cv::Point scanStart, scanEnd;
	cv::Mat edges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat scanROI;
	double heightThresh = -5;
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	Coords scanPosFbk;
	edgeMsg msg;
	double posErrThr = 2.5; // position error threshold for how close the current position is to the target
	cv::Point2d curPos;

	while (segmentNumScan < segments.size()){
		if (collectData(handle, DCCHandle, &collectedData[0][0])) {
			// Trigger the scanner and collect the scanner data
			getScan(collectedData, &scanPosFbk, scan);
			curPos = cv::Point2d(scanPosFbk.x, scanPosFbk.y);

			// Find the part of the scan that is within the ROI of the print
			if (scan2ROI(scan, scanPosFbk, raster.roi(), raster.size(), scanROI, scanStart, scanEnd)){
				// Finding the edges
				findEdges(raster.boundaryMask(), scanStart, scanEnd, scanROI, edges, heightThresh);
			}		
		
			// compare the current position to the scanDonePt of the segment
			if (cv::norm(curPos - segments[segmentNumScan].scanDonePt()) < posErrThr) {
				std::cout << "Segment " << segmentNumScan << " scanned. Sending data for processing." << std::endl;
				// Check if this was the last segmet to scan
				msg.addEdges(edges, segmentNumScan, (segmentNumScan == segments.size()-1) );
				// push the edges to the error calculating thread
				q_edgeMsg.push(msg);
				// move to next segment
				segmentNumScan++;
			}
		}
		else { A3200Error(); }
	}
	// Save the data
	writeCSV(outDir + "edges.csv", edges);
	cv::Mat image = cv::Mat::zeros(raster.size(), CV_8UC3);
	image = showRaster(raster.draw(), raster.boundaryMask(), edges, cv::Scalar(0, 0, 255), MM2PIX(0.2));
	cv::imwrite(outDir + "edges.png", image);
	std::cout << "All segments have been scanned. Ending scanning thread." << std::endl;
}

void t_GetMatlErrors(Raster raster) {
	edgeMsg msg;
	std::vector<cv::Point> centerline;
	std::vector<cv::Point> lEdgePts, rEdgePts;
	std::vector<double> errCL, errWD;
	double targetWidth = .5;
	bool doneScanning = false;

	while (!doneScanning){
		// wait for the message to be pushed from the scanning thread
		q_edgeMsg.wait_and_pop(msg);
		doneScanning = msg.doneScanning();
		segmentNumError = msg.segmentNum();
		// find and smooth the right and left edges
		getMatlEdges(segments[msg.segmentNum()].ROI(), msg.edges(), lEdgePts, rEdgePts);
		segments[msg.segmentNum()].addEdges(lEdgePts, rEdgePts);
		// Calculate errors
		centerline = segments[msg.segmentNum()].centerline();
		getMatlErrors(centerline, targetWidth, raster.size(), segments[msg.segmentNum()].lEdgePts(), segments[msg.segmentNum()].rEdgePts(), errCL, errWD);
		segments[msg.segmentNum()].addErrors(errCL, errWD, centerline);
	}
	cv::Mat image;
	showErrors(raster.draw(), image, segments);
	cv::imwrite(outDir + "errors.png", image);
	std::cout << "All segments have been processed. Ending error processing thread." << std::endl;
}

/**
 * @brief 
 * @param rate rate at which to poll the position in [ms]
*/
void t_PollPositionFeedback(int rate) {
	// setup polling of X and Y position feedback
	WORD itemIndexArray[] = { AXISINDEX_00, AXISINDEX_01 };
	STATUSITEM itemCodeArray[] = { STATUSITEM_PositionFeedback, STATUSITEM_PositionFeedback };
	DWORD itemExtrasArray[] = { 0, 0 };
	double itemValuesArray[2];
	cv::Point2d curPos;

	while (true) {
		// get the position feedback simultaneously
		if (!A3200StatusGetItems(handle, 2, itemIndexArray, itemCodeArray, itemExtrasArray, itemValuesArray)) {
			curPos = cv::Point2d(itemValuesArray[0], itemValuesArray[1]);

		} 
		else { A3200Error(); }
		std::this_thread::sleep_for(std::chrono::milliseconds(rate));
	}

}