#include <iostream>
#include <algorithm>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "A3200.h"
#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "scanning.h"
#include "draw.h"
#include "errors.h"
#include "A3200_functions.h"
#include "csvMat.h"
#include "raster.h"
#include "print.h"

void t_CollectScans(Raster raster) {
	cv::Mat scan(1, NUM_DATA_SAMPLES, CV_64F);
	cv::Point scanStart, scanEnd;
	cv::Mat edges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat scanROI;
	double heightThresh = -3;
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	Coords scanPosFbk;
	edgeMsg msg;
	double posErrThr = 2.5; // position error threshold for how close the current position is to the target
	cv::Point2d curPos;
	int locXoffset = 0;

	// wait for pre-print to complete before starting the scanner
	q_scanMsg.wait_and_pop();

	while (segNumScan < segments.size()){
		if (collectData(handle, DCCHandle, &collectedData[0][0])) {
			// Trigger the scanner and collect the scanner data
			if (getScan(collectedData, &scanPosFbk, scan, locXoffset)){
				// Find the part of the scan that is within the ROI of the print
				if (scan2ROI(scan, scanPosFbk, locXoffset, raster.roi(), raster.size(), scanROI, scanStart, scanEnd)) {
					// Finding the edges
					findEdges2(raster.boundaryMask(), scanStart, scanEnd, scanROI, edges);
				}
			}
			// compare the current position to the scanDonePt of the segment
			curPos = cv::Point2d(scanPosFbk.x, scanPosFbk.y);
			if (cv::norm(curPos - segments[segNumScan].scanDonePt()) < posErrThr) {
				std::cout << "Segment " << segNumScan << " scanned. Sending data for processing." << std::endl;
				// Check if this was the last segmet to scan
				msg.addEdges(edges, segNumScan, (segNumScan == segments.size() - 1));
				// push the edges to the error calculating thread
				q_edgeMsg.push(msg);
				// move to next segment
				segNumScan++;
			}
			
		}
		else { A3200Error(); }
	}
	// Save the data
	cv::Mat image = cv::Mat::zeros(raster.size(), CV_8UC3);
	raster.draw(image, image);
	raster.drawBdry(image, image, cv::Scalar(255, 0, 0));
	drawEdges(image, image, edges, cv::Scalar(0, 0, 255), MM2PIX(0.1));
	cv::imwrite(outDir + "edges.png", image);
	cv::imwrite(outDir + "edgedata.png", edges);
	std::cout << "All segments have been scanned. Ending scanning thread." << std::endl;
}

void t_GetMatlErrors(Raster raster, std::vector<std::vector<Path>> path) {
	edgeMsg inMsg;
	errsMsg outMsg;
	std::vector<cv::Point> waypoints;
	std::vector<cv::Point> lEdgePts, rEdgePts;
	std::vector<double> errCL, errWD, targetWidths;
	bool doneScanning = false;

	while (!doneScanning){
		// wait for the message to be pushed from the scanning thread
		q_edgeMsg.wait_and_pop(inMsg);
		doneScanning = inMsg.doneScanning();
		segNumError = inMsg.segmentNum();
		
		// find and smooth the right and left edges
		getMatlEdges(segments[segNumError].ROI(), inMsg.edges(), lEdgePts, rEdgePts);
		// If there are edge points, calculate errors
		if (!lEdgePts.empty() && !rEdgePts.empty()) {
			std::for_each(path[segNumError].begin(), path[segNumError].end(), [&targetWidths](Path& pth) {targetWidths.push_back(pth.w); });
			waypoints = segments[segNumError].waypoints();
			getErrorsAt(waypoints, targetWidths, raster.size(), lEdgePts, rEdgePts, errCL, errWD);
		}
		// Push the errors to the controller
		std::cout << "Segment " << segNumError << " errors processed. Sending data to controller." << std::endl;
		outMsg.addErrors(errCL, errWD, segNumError);
		q_errsMsg.push(outMsg);

		//TODO: store the errors someplace else?
		segments[segNumError].addEdges(lEdgePts, rEdgePts);
		segments[segNumError].addErrors(errCL, errWD);
		// clear the errors
		errCL.clear(); 
		errWD.clear();
	}
	cv::Mat image = cv::Mat::zeros(raster.size(), CV_8UC3);
	raster.draw(image, image);
	//raster.drawBdry(image, image, cv::Scalar(255, 0, 0));
	//drawErrors(image, image, segments);
	drawMaterial(image, image, segments, path);
	addScale(image, 1, cv::Point(5, 15));
	cv::imwrite(outDir + "errors.png", image);
	std::cout << "All segments have been processed. Ending error processing thread." << std::endl;
}

void t_controller(std::vector<std::vector<Path>> path, int segsBeforeCtrl) {
	errsMsg inMsg;
	pathMsg outMsg;
	int segNum = 0;

	while (segNum < path.size()) {
		// do not modify the initial segment inputs
		if (segNum >= segsBeforeCtrl) {
			q_errsMsg.wait_and_pop(inMsg);
			// Use the errors from the previous segment to calculate the control next segment 
			segNum = inMsg.segmentNum() + 3;
			// If no errors were calculated, do not modify the path
			if (!inMsg.errCL().empty() && !inMsg.errWD().empty()) {
				// MODIFY PATH INPUT HERE
			}
		}
		// Send path coords to queue
		outMsg.addPath(path[segNum], segNum);
		q_pathMsg.push(outMsg);
		segNum++;
	}
	std::cout << "Ending controller thread" << std::endl;
}

void t_printQueue(Path firstWpt, bool extrude) {
	pathMsg inMsg;
	int segNum = -1;
	double queueLineCount;
	
	// End any program already running
	if (!A3200ProgramStop(handle, TASK_PRINT)) { A3200Error(); }
	if (!A3200MotionEnable(handle, TASK_PRINT, AXES_ALL)) { A3200Error(); }

	// Run pre-print process
	prePrint(firstWpt);
	std::cout << "Pre-print complete" << std::endl;
	
	// notify scanner to start
	q_scanMsg.push(true);

	// Put task into Queue mode and then pause the program while queue is loaded.
	if (!A3200ProgramInitializeQueue(handle, TASK_PRINT)) { A3200Error(); }
	if (!A3200ProgramPause(handle, TASK_PRINT)) { A3200Error(); }

	// Set VEOLCITY mode ON and put task in absolute mode 
	if (!A3200CommandExecute(handle, TASK_PRINT, (LPCSTR)"VELOCITY ON\nG90\n", NULL)) { A3200Error(); }
	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }

	if (extrude) {
		// Enable the extruder
		extruder.enable();
	}

	// Fill the command queue with the path
	while (static_cast<__int64>(segNum) + 1 < segments.size()) {
		// wait for the path coordinates to be pushed
		q_pathMsg.wait_and_pop(inMsg);
		segNum = inMsg.segmentNum();

		for (auto it = inMsg.path().begin(); it != inMsg.path().end(); ++it) {
			auto path = *(it);
			while (!A3200CommandExecute(handle, TASK_PRINT, path.cmd(), NULL)) {
				// If the command failed to load into the queue
				if (A3200GetLastError().Code == ErrorCode_QueueBufferFull) {
					// Wait if the Queue is full.
					std::cout << "Queue is full" << std::endl;
					Sleep(10);
				}
				else {
					A3200Error();
					break;
				}
			}
		}
		std::cout << "Segment " << segNum << " loaded" << std::endl;
		if (!A3200CommandExecute(handle, TASK_PRINT, std::string("MSGDISPLAY 0, \"Segment " + std::to_string(segNum) + " printed\" \n").c_str(), NULL)) { A3200Error(); }
		if (!A3200CommandExecute(handle, TASK_PRINT, std::string("MSGLAMP 1, YELLOW,\"Segment " + std::to_string(segNum) + " printed\"\n").c_str(), NULL)) { A3200Error(); }

		if (!A3200ProgramStart(handle,TASK_PRINT)) { A3200Error(); }
	}
	// Run post-print process
	postPrint();
	if (!A3200MotionDisable(handle, TASK_PRINT, AXES_ALL)) { A3200Error(); }

	// Periodically check to see if the queue is empty
	std::cout << "Print loaded. Waiting for queue to empty." << std::endl;
	if (!A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }
	while (0 != (int)queueLineCount) {
		Sleep(10);
		if (!A3200StatusGetItem(handle, TASKID_01, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }
	}
	// Stop using queue mode
	if (!A3200ProgramStop(handle, TASK_PRINT)) { A3200Error(); }

	std::cout << "Printing Complete. Ending printing thread." << std::endl;
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