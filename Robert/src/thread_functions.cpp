#include "thread_functions.h"
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
#include "controller.h"

void t_CollectScans(Raster raster) {
	cv::Mat scan(1, NUM_DATA_SAMPLES, CV_64F);
	cv::Point scanStart, scanEnd;
	cv::Mat scanROI;
	double heightThresh = -3;
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	Coords scanPosFbk;
	edgeMsg msg;
	double posErrThr = 2.5; // position error threshold for how close the current position is to the target
	cv::Point2d curPos;
	int locXoffset = 0;
	int layer = segments.front().layer();
	cv::Mat edges = cv::Mat::zeros(raster.size(layer), CV_8UC1);
	std::vector<cv::Mat> pastEdges;

	// wait for pre-print to complete before starting the scanner
	q_scanMsg.wait_and_pop();

	segNumScan = 0;
	while (segNumScan < segments.size()){
		// if there was a layer change, clear all the edges
		if (segments[segNumScan].layer() != layer) {
			layer = segments[segNumScan].layer();
			pastEdges.push_back(edges);
			edges = cv::Mat::zeros(raster.size(layer), CV_8UC1);
		}
		if (collectData(handle, DCCHandle, &collectedData[0][0])) {
			// Trigger the scanner and collect the scanner data
			if (getScan(collectedData, &scanPosFbk, scan, locXoffset)){
				// Find the part of the scan that is within the ROI of the print
				if (scan2ROI(scan, scanPosFbk, locXoffset, raster.roi(layer), raster.size(layer), scanROI, scanStart, scanEnd)) {
					// Finding the edges
					findEdges2(raster.boundaryMask(layer), scanStart, scanEnd, scanROI, edges);
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
	pastEdges.push_back(edges);
	cv::Mat image;
	for (int i = 0; i < pastEdges.size(); i++) 
	{
		image = cv::Mat::zeros(raster.size(i + segments.front().layer()), CV_8UC3);
		raster.draw(image, image, i + segments.front().layer());
		raster.drawBdry(image, image, i + segments.front().layer(), cv::Scalar(255, 0, 0), MM2PIX(0.05));
		drawEdges(image, image, pastEdges[i], cv::Scalar(0, 0, 255), MM2PIX(0.1));
		cv::imwrite(outDir + "edges_" + std::to_string(i + segments.front().layer()) + ".png", image);
		cv::imwrite(outDir + "edgedata_" + std::to_string(i + segments.front().layer()) + ".png", pastEdges[i]);
	}
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
		getMatlEdges(segments[segNumError].ROI(), segments[segNumError].dir(), inMsg.edges(), lEdgePts, rEdgePts);
		// If there are edge points, calculate errors
		if (!lEdgePts.empty() && !rEdgePts.empty()) {
			std::for_each(path[segNumError].begin(), path[segNumError].end(), [&targetWidths](Path& pth) {targetWidths.push_back(pth.w); });
			waypoints = segments[segNumError].waypoints();
			getErrorsAt(waypoints, targetWidths, raster.size(segments[segNumError].layer()), lEdgePts, rEdgePts, errCL, errWD);
		}

		// Store the errors in the segment class
		segments[segNumError].addEdges(lEdgePts, rEdgePts);
		segments[segNumError].addErrors(errCL, errWD);

		// replace the NAN error values with the error at the adjacent point
		if (std::count_if(errWD.begin(), errWD.end(), [](double& a) {return !std::isnan(a); }) > 0) // check that the vector isn't all nan
		{
			for (auto it = errWD.begin(); it != errWD.end(); ++it) {
				if (std::isnan(*it)) {
					//FIXME: Error thrown if vector is all nan 
					*it = it - errWD.begin() < errWD.end() - it ?
						*std::find_if(errWD.begin(), errWD.end(), [](double& a) {return !std::isnan(a); }) : *std::find_if(errWD.rbegin(), errWD.rend(), [](double& a) {return !std::isnan(a); });
				}
			}
		}

		// Push the errors to the controller
		std::cout << "Segment " << segNumError << " errors processed. Sending data to controller." << std::endl;
		outMsg.addErrors(errCL, errWD, segNumError);
		q_errsMsg.push(outMsg);

		// clear the errors
		errCL.clear(); 
		errWD.clear();
		targetWidths.clear();
	}
	// Save the data

	// drawing the filtered edges
	int layer = segments.front().layer();
	cv::Mat filteredEdges = cv::Mat::zeros(raster.size(layer), CV_8UC1);
	cv::Mat image = cv::Mat::zeros(raster.size(layer), CV_8UC3);
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		if ((*it).layer() != layer) {
			cv::imwrite(outDir + "edgedata_filt_" + std::to_string(layer) + ".png", filteredEdges);
			raster.draw(image, image, layer);
			raster.drawBdry(image, image, layer, cv::Scalar(255, 0, 0), MM2PIX(0.05));
			drawEdges(image, image, filteredEdges, cv::Scalar(0, 0, 255), MM2PIX(0.1));
			cv::imwrite(outDir + "edges_filt_" + std::to_string(layer) + ".png", image);
			layer = (*it).layer();
			filteredEdges = cv::Mat::zeros(raster.size(layer), CV_8UC1);
			image = cv::Mat::zeros(raster.size(layer), CV_8UC3);
		}
		for (auto it2 = (*it).lEdgePts().begin(); it2 != (*it).lEdgePts().end(); ++it2) { filteredEdges.at<uchar>((*it2)) = 255; }
		for (auto it2 = (*it).rEdgePts().begin(); it2 != (*it).rEdgePts().end(); ++it2) { filteredEdges.at<uchar>((*it2)) = 255; }
	}
	cv::imwrite(outDir + "edgedata_filt_" + std::to_string(layer) + ".png", filteredEdges);
	raster.draw(image, image, layer);
	raster.drawBdry(image, image, layer, cv::Scalar(255, 0, 0), MM2PIX(0.05));
	drawEdges(image, image, filteredEdges, cv::Scalar(0, 0, 255), MM2PIX(0.1));
	cv::imwrite(outDir + "edges_filt_" + std::to_string(layer) + ".png", image);

	// drawing the material
	for (int i = segments.front().layer(); i <= layer ; i++)
	{
		image = cv::Mat::zeros(raster.size(i), CV_8UC3);
		raster.draw(image, image, i);
		drawMaterial(image, image, segments, path, i);
		addScale(image, 1, cv::Point(5, 25), 2);
		cv::imwrite(outDir + "matl_" + std::to_string(i) + ".png", image);
	}

	std::cout << "All segments have been processed. Ending error processing thread." << std::endl;
}

void t_noController(std::vector<std::vector<Path>> path) {
	errsMsg inMsg;
	pathMsg outMsg;
	int nextSeg = 0;

	while (nextSeg < path.size()) {
		// Send path coords to queue
		outMsg.addPath(path[nextSeg], nextSeg);
		q_pathMsg.push(outMsg);
		nextSeg++;
	}
	std::cout << "Ending controller thread" << std::endl;
}

void t_controller(std::vector<std::vector<Path>>& path, Controller& controller) {
	errsMsg inMsg;
	pathMsg outMsg;
	int nextSeg = 0;
	int segStartCtrl = 4;

	while (nextSeg < path.size()) {
		// do not modify the initial segment inputs
		if (nextSeg >= segStartCtrl) {
			q_errsMsg.wait_and_pop(inMsg);
			// Use the errors from the previous segment to calculate the control next segment 
			nextSeg = inMsg.segmentNum() + segStartCtrl;
			// If errors were calculated, modify the path
			if (!inMsg.errCL().empty() && !inMsg.errWD().empty()) {
				for (int i = 0; i < inMsg.errWD().size(); i++) {
					controller.nextPath(path[nextSeg][i], path[inMsg.segmentNum()][i], inMsg.errWD()[i], inMsg.errCL()[i]);
				}
			}
		}
		// Send path coords to queue
		outMsg.addPath(path[nextSeg], nextSeg);
		q_pathMsg.push(outMsg);
		nextSeg++;
	}
	std::cout << "Ending controller thread" << std::endl;
}

void t_printQueue(Path firstWpt, PrintOptions printOpts) {
	pathMsg inMsg;
	int segNum = -1;
	int layer = segments.front().layer();
	int layerCt = 0;
	double queueLineCount, queueSize;
	bool programStarted = false;
	double asyncThetaPos = 90.0 * (double)segments.front().dir();
	if (printOpts.asyncTheta > 0) { firstWpt.T = asyncThetaPos; }

	// End any program already running
	if (!A3200ProgramStop(handle, TASK_PRINT)) { A3200Error(); }
	if (!A3200MotionEnable(handle, TASK_PRINT, AXES_ALL)) { A3200Error(); }

	// Run pre-print process
	prePrint(firstWpt, printOpts);
	std::cout << "Pre-print complete" << std::endl;
	
	// notify scanner to start
	q_scanMsg.push(true);
	
	// Put task into Queue mode and then pause the program while queue is loaded.
	if (!A3200ProgramInitializeQueue(handle, TASK_PRINT)) { A3200Error(); }
	if (!A3200ProgramPause(handle, TASK_PRINT)) { A3200Error(); }

	// Set VEOLCITY mode ON and put task in absolute mode 
	if (!A3200CommandExecute(handle, TASK_PRINT, (LPCSTR)"VELOCITY ON\nG90\n", NULL)) { A3200Error(); }
	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }

	// Enable the extruder
	if (printOpts.extrude) { extruder.enable(); }

	// Get the size of the queue buffer
	if (!A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCapacity, 0, &queueSize)) { A3200Error(); }
	// Get the number of items that have been loaded in the queue
	if (!A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }

	// Fill the command queue with the path
	while (static_cast<__int64>(segNum) + 1 < segments.size()) {
		// wait for the path coordinates to be pushed
		q_pathMsg.wait_and_pop(inMsg);
		segNum = inMsg.segmentNum();
		// Check if new layer
		if (segments[segNum].layer() != layer) {
			layer = segments[segNum].layer();
			layerCt++;
			asyncThetaPos = 90.0 * (double)segments[static_cast<__int64>(segNum)].dir();
			while (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_03), asyncThetaPos, printOpts.asyncTheta)) {
				if (A3200GetLastError().Code == ErrorCode_QueueBufferFull) { Sleep(10); }
				else { A3200Error(); break; }
			}
			//TODO: add rotation for layer change
		}
		// if printing with asynchronous theta movement, check if it's an odd segment 
		if (printOpts.asyncTheta > 0 && ((segNum + layerCt) % 2 == 1)) {
			// Set the new angle based on the direction of the segment before
			asyncThetaPos = 90.0 * (double)((segments[static_cast<__int64>(segNum) - 1].dir() + 2) % 4);
			while (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_03), asyncThetaPos, printOpts.asyncTheta)) {
				if (A3200GetLastError().Code == ErrorCode_QueueBufferFull) { Sleep(10); }
				else { A3200Error(); break; }
			}
			queueLineCount += 1;
		}

		for (auto it = inMsg.path().begin(); it != inMsg.path().end(); ++it) {
			auto path = *(it);
			while (!A3200CommandExecute(handle, TASK_PRINT, path.cmd(printOpts.asyncTheta == 0), NULL)) {
				// If the command failed to load into the queue
				if (A3200GetLastError().Code == ErrorCode_QueueBufferFull) {
					// Wait if the Queue is full.
					Sleep(10);
				}
				else {
					A3200Error();
					break;
				}
			}
			queueLineCount += 2;
			// if the queue is almost full, start the program
			if ((queueLineCount > (queueSize - 10)) && !programStarted) {
				if (!A3200ProgramStart(handle, TASK_PRINT)) { A3200Error(); }
				else { programStarted = true; }
			}
		}
		std::cout << "Segment " << segNum << " loaded" << std::endl;
		//if (!A3200CommandExecute(handle, TASK_PRINT, std::string("MSGDISPLAY 0, \"Segment " + std::to_string(nextSeg) + " printed\" \n").c_str(), NULL)) { A3200Error(); }
		//if (!A3200CommandExecute(handle, TASK_PRINT, std::string("MSGLAMP 1, YELLOW,\"Segment " + std::to_string(nextSeg) + " printed\"\n").c_str(), NULL)) { A3200Error(); }

		if (!programStarted){
			if (!A3200ProgramStart(handle, TASK_PRINT)) { A3200Error(); }
			else { programStarted = true; }
		}
	}
	// wait until there is room in the queue to load the post print 
	if (!A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }
	while (queueLineCount > (queueSize - 100)) {
		Sleep(10);
		if (!A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }
	}
	// Load post-print process
	postPrint(inMsg.path().back(), printOpts);
	if (!A3200MotionDisable(handle, TASK_PRINT, AXES_ALL)) { A3200Error(); }

	// Periodically check to see if the queue is empty
	std::cout << "Print loaded. Waiting for queue to empty." << std::endl;
	if (!A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }
	while (0 != (int)queueLineCount) {
		Sleep(10);
		if (!A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }
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