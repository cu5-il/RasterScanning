#include "myGlobals.h"
#include "constants.h"
#include "A3200.h"
#include <iostream>
#include <cmath>
#include <deque>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

bool setupDataCollection(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle) {
	// Adding the signals to be collected
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_AnalogInput0, AXISINDEX_00, 0)) { return false; }		// AI0
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_AnalogOutput0, AXISINDEX_00, 0)) { return false; }	// AO0
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_PositionFeedback, AXISINDEX_00, 0)) { return false; } // X-axis position feedback
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_PositionFeedback, AXISINDEX_01, 0)) { return false; } // Y-axis position feedback
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_PositionFeedback, AXISINDEX_02, 0)) { return false; } // Z-axis position feedback
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_PositionFeedback, AXISINDEX_03, 0)) { return false; } // T-axis position feedback

	// Setting the parameters for the data collection
	if (!A3200DataCollectionConfigSetPeriod(DCCHandle, SAMPLING_TIME)) { return false; }
	if (!A3200DataCollectionConfigSetSamples(DCCHandle, NUM_DATA_SAMPLES)) { return false; }

	// Apply the data collection configuration to the A3200 
	if (!A3200DataCollectionConfigApply(handle, DCCHandle)) { return false; }

	// Initializing AnalogOutput0 to 0V 
	if (!A3200IOAnalogOutput(handle, TASK_SCAN, 0, AXISINDEX_00, 0)) { return false; }

	return true;
}

bool collectData(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle, DOUBLE* data) {
	// Start the data collection
	if (!A3200DataCollectionStart(handle, DCCHandle)) { return false; }

	// Triggering the laser scanner by sending a pulse from AnalogOutput0 
	if (!A3200IOAnalogOutput(handle, TASK_SCAN, 0, AXISINDEX_00, OUT_VOLTAGE)) { return false; }
	if (!A3200IOAnalogOutput(handle, TASK_SCAN, 0, AXISINDEX_00, 0)) { return false; }

	// Retrieving the collected data
	if (!A3200DataCollectionDataRetrieve(handle, NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, (DOUBLE*)data)) { return false; }

	return true;
}

void getScan(double data[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& scan) {
	int fbIdx[2];
	int scanStartIdx;
	cv::Mat scanVoltage_8U, scanEdges, scanEdgesIdx;

	cv::Mat dataMat(NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, CV_64F, data); // copying the collected data into a matrix

	// Get the position feedback when the laser was triggered
	//NOTE: feedback values are given as counts and can be converted using the CountsPerUnit Parameter in the A3200 software
	cv::minMaxIdx(dataMat.row(1), NULL, NULL, fbIdx, NULL); //find the rising edge of the trigger signal sent to the laser
	fbk->x = dataMat.at<double>(2, fbIdx[1]) / -1000; // assigning the position feedback values
	fbk->y = dataMat.at<double>(3, fbIdx[1]) / 1000;
	fbk->z = dataMat.at<double>(4, fbIdx[1]) / 10000;
	fbk->T = dataMat.at<double>(5, fbIdx[1]) * 360 / 200000;

	// Finding the voltage header of the scanner signal, i.e find the start of the scanned profile
	// Search only the data after the triggering signal was sent (after index fbIdx[1])
	cv::normalize(dataMat(cv::Range(0, 1), cv::Range(fbIdx[1], dataMat.cols)), scanVoltage_8U, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::Canny(scanVoltage_8U, scanEdges, 10, 30, 3);
	cv::findNonZero(scanEdges, scanEdgesIdx);

	scanStartIdx = scanEdgesIdx.at<int>(1, 0) + fbIdx[1]; // Starting index of the scan profile
	// Check if entire scan captured
	if ((scanStartIdx + NUM_PROFILE_PTS) <= NUM_DATA_SAMPLES) {
		scan = dataMat(cv::Rect(scanStartIdx, 0, NUM_PROFILE_PTS, 1)).clone() / OPAMP_GAIN; // Isolating the scanned profile and converting to height
	}
	else {
		//HACK: if entire scan isn't captured, send junk data
		scan = cv::Mat(1, NUM_PROFILE_PTS, CV_64F, -11).clone();
	}

	return;
}

bool scan2ROI(cv::Mat& scan, const Coords fbk, const cv::Rect2d printROI, cv::Size rasterSize, cv::Mat& scanROI, cv::Point& scanStart, cv::Point& scanEnd) {
	//TODO: Convert printROI from mm to pixels
	cv::Point2d XY_start, XY_end;
	double X, Y;
	double local_x;
	int startIdx = -1, endIdx = -1;

	for (int i = 0; i < scan.cols; i++) {
		// Local coordinate of the scanned point (with respect to the scanner)
		local_x = -SCAN_WIDTH / 2 + i * SCAN_WIDTH / (int(scan.cols) - 1);
		// Transforming local coordinate to global coordinates
		X = fbk.x + SCAN_OFFSET_X * cos(fbk.T * PI / -180) - (local_x + SCAN_OFFSET_Y) * sin(fbk.T * PI / -180);
		Y = fbk.y + SCAN_OFFSET_X * sin(fbk.T * PI / -180) + (local_x + SCAN_OFFSET_Y) * cos(fbk.T * PI / -180);
		// Check if scanned point in outside the print ROI 
		if (!printROI.contains(cv::Point2d(X, Y))) {
		}
		else if (startIdx == -1) {
			startIdx = i;
			XY_start = cv::Point2d(X, Y);
		}
		else {
			endIdx = i;
			XY_end = cv::Point2d(X, Y);
		}
	}

	// Check to see if the scan was in the ROI
	if ((startIdx != -1) && (endIdx != -1)) {
		//convert the start and end (X,Y) coordinates of the scan to points on the image
		scanStart = cv::Point(MM2PIX(XY_start.x - printROI.tl().x), MM2PIX(XY_start.y - printROI.tl().y));
		scanEnd = cv::Point(MM2PIX(XY_end.x - printROI.tl().x), MM2PIX(XY_end.y - printROI.tl().y));
		cv::Range scanROIRange = cv::Range(startIdx, endIdx);

		// Interpolate scan so it is the same scale as the raster reference image
		cv::LineIterator it(rasterSize, scanStart, scanEnd, 8); // make a line iterator between the start and end points of the scan
		cv::resize(scan.colRange(scanROIRange), scanROI, cv::Size(it.count, scan.rows), cv::INTER_LINEAR);
		return true;
	}
	else {
		//TODO: Remove outputting junk scan start and end points
		scanStart = cv::Point(-1, -1);
		scanEnd = cv::Point(-1, -1);
		return false;
	}

	return false;
}

void findEdges(cv::Mat edgeBoundary, cv::Point scanStart, cv::Point scanEnd, cv::Mat& scanROI, cv::Mat& edges, double heightThresh) {

	if ((scanStart != cv::Point(-1, -1)) && (scanEnd != cv::Point(-1, -1))) //Check if scan is within ROI
	{
		cv::LineIterator lineit(edgeBoundary, scanStart, scanEnd, 8);
		std::deque<int> windowPts;
		uchar lastVal = 0;
		uchar curVal = 0;
		int numRising = 0, numFalling = 0;
		cv::Point2d edgeCoord, slope;

		// Initialize local masks to zero
		//cv::Mat locEdges = cv::Mat::zeros(scanROI.size(), CV_8U);
		//cv::Mat locWin = cv::Mat::zeros(scanROI.size(), CV_8U);

		// find the intersection of the scan and the edge boundary using a line iterator
		for (int i = 0; i < lineit.count; i++, ++lineit) {
			curVal = *(const uchar*)*lineit;
			if ((curVal == 255) && (lastVal == 0) && (i != 0)) { // find rising edges
				windowPts.push_back(i);
				numRising++;
			}
			if ((curVal == 0) && (lastVal == 255) && (i != 0) && (numRising > 0)) { // find falling edges
				windowPts.push_back(i);
				numFalling++;
			}
			lastVal = curVal;
		}
		// Check if equal number of rising and falling edges (i.e. an odd number of window points)
		if ((windowPts.size() % 2) != 0) {
			// if scan ends in the middle of a rod, remove the last point; Otherwise, scan start in the middle of a rod so remove the first point
			if (numRising > numFalling) { windowPts.pop_back(); }
			else { windowPts.pop_front(); }
		}

		// create a height mask for the scan profile to remove all edges below a height threshold
		cv::Mat heightMask;
		cv::threshold(scanROI, heightMask, heightThresh, 1, cv::THRESH_BINARY);
		cv::normalize(heightMask, heightMask, 0, 255, cv::NORM_MINMAX, CV_8U);

		// Search within the edges of the dialated raster for the actual edges
		cv::Range searchRange;
		// Take derivative and blur
		cv::Mat dx, ROIblur;
		int aperture_size = 7;
		int sigma = 61;
		int sz = 19;
		cv::GaussianBlur(scanROI, ROIblur, cv::Size(sz, sz), (double)sigma / 10);
		cv::Sobel(ROIblur, dx, -1, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
		int foundEdges[2];
		int maxIdx[2];
		int minIdx[2];

		// loop through all the search windows
		for (auto it = windowPts.begin(); it != windowPts.end(); std::advance(it, 2)) {
			// set the window search range
			searchRange = cv::Range(*it, *std::next(it));
			//find the edges by finding the local extrema of the profile derivative 
			cv::minMaxIdx(dx(cv::Range::all(), searchRange), NULL, NULL, minIdx, maxIdx);
			foundEdges[0] = maxIdx[1] + searchRange.start;
			foundEdges[1] = minIdx[1] + searchRange.start;

			// mark edges on local profile and global ROI
			slope = cv::Point2d(scanEnd - scanStart) / lineit.count;
			for (int j = 0; j < 2; j++) {
				edgeCoord = cv::Point2d(scanStart) + foundEdges[j] * slope;
				// check if edges are within height mask
				if (heightMask.at<uchar>(cv::Point(foundEdges[j], 0)) == 255) {
					//locEdges.at<uchar>(cv::Point(foundEdges[j], 0)) = 255;
					edges.at<uchar>(cv::Point2i(edgeCoord)) = 255;
				}
			}
			// mark window borders
			//locWin.at<uchar>(cv::Point(searchRange.start, 0)) = 255;
			//locWin.at<uchar>(cv::Point(searchRange.end, 0)) = 255;
		}
	}

	return;
}