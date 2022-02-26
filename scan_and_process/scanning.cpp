#include "myGlobals.h"
#include "constants.h"
#include "A3200.h"
#include <iostream>
#include <cmath>
#include <deque>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

//for debugging scanning
//#include <opencv2/highgui.hpp>
//#include <CvPlot/cvplot.h>

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

bool getScan(double data[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& scan, int &locXoffset) {
	int fbIdx[2], voltHead[2];
	int scanStartIdx, scanEndIdx;
	cv::Mat scanVoltage_8U, scanEdges;
	std::vector<cv::Point> scanEdgesIdx;
	cv::Mat morph, kern;
	cv::Mat dataMat(NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, CV_64F, data); // copying the collected data into a matrix
	int scanXtrunc = 20;

	// Get the position feedback when the laser was triggered
	//NOTE: feedback values are given as counts and can be converted using the CountsPerUnit Parameter in the A3200 software
	cv::minMaxIdx(dataMat.row(1), NULL, NULL, fbIdx, NULL); //find the rising edge of the trigger signal sent to the laser
	fbk->x = dataMat.at<double>(2, fbIdx[1]) / -1000; // assigning the position feedback values
	fbk->y = dataMat.at<double>(3, fbIdx[1]) / 1000;
	fbk->z = dataMat.at<double>(4, fbIdx[1]) / 10000;
	fbk->T = dataMat.at<double>(5, fbIdx[1]) * 360 / 200000;

	scan = dataMat(cv::Range(0, 1), cv::Range(fbIdx[1], dataMat.cols)).clone();

	// check if the scan voltage goes below 3V to verify a scan was sent
	if (!cv::checkRange(dataMat.row(0), true, (cv::Point*)0, 3, DBL_MAX)) {
		// Finding the voltage header of the scanner signal, i.e find the start of the scanned profile
		// Search only the data after the triggering signal was sent (after index fbIdx[1])
		kern = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(scan, morph, cv::MORPH_BLACKHAT, kern, cv::Point(-1, -1), 2);
		cv::minMaxIdx(morph, NULL, NULL, NULL, voltHead);

		// Isolating the scanned profile and converting to height
		cv::normalize(scan, scanVoltage_8U, 0, 255, cv::NORM_MINMAX, CV_8U);
		cv::threshold(scanVoltage_8U, scanEdges, 2, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
		cv::morphologyEx(scanEdges, scanEdges, cv::MORPH_DILATE, kern, cv::Point(-1, -1), 2);
		cv::Sobel(scanEdges, scanEdges, -1, 2, 0, 3, 1, 0, cv::BORDER_REPLICATE);
		cv::findNonZero(scanEdges, scanEdgesIdx);
		// Check if entire scan captured
		if (scanEdgesIdx.size() == 2) {
			locXoffset = scanEdgesIdx[0].x - voltHead[1] + scanXtrunc;
			scanStartIdx = fbIdx[1] + scanEdgesIdx[0].x + scanXtrunc;
			scanEndIdx = fbIdx[1] + scanEdgesIdx[1].x;
		}
		else if (scanEdgesIdx.size() == 1) {
			locXoffset = scanEdgesIdx[0].x - voltHead[1] + scanXtrunc;
			scanStartIdx = fbIdx[1] + scanEdgesIdx[0].x + scanXtrunc;
			scanEndIdx = dataMat.cols;
		}
		else
			return false;
		
		scan = dataMat(cv::Range(0, 1), cv::Range(scanStartIdx, scanEndIdx)).clone() / OPAMP_GAIN;
		return true;
	}
	else
		return false;
}

bool scan2ROI(cv::Mat& scan, const Coords fbk, const int locXoffset, const cv::Rect2d printROI, cv::Size rasterSize, cv::Mat& scanROI, cv::Point& scanStart, cv::Point& scanEnd) {
	cv::Point2d XY_start, XY_end;
	double X, Y;
	double local_x;
	int startIdx = -1, endIdx = -1;
	double dx = SCAN_WIDTH / (NUM_PROFILE_PTS - 1);

	for (int i = 0; i < scan.cols; i++) {
		// Local coordinate of the scanned point (with respect to the scanner)
		local_x = -SCAN_WIDTH / 2 + dx * (i + locXoffset) + SCAN_OFFSET_Y;
		// Transforming local coordinate to global coordinates
		X = fbk.x + SCAN_OFFSET_X * cos(fbk.T * PI / -180) - (local_x) * sin(fbk.T * PI / -180);
		Y = fbk.y + SCAN_OFFSET_X * sin(fbk.T * PI / -180) + (local_x) * cos(fbk.T * PI / -180);
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
		if (it.count == 0) {
			return false;
		}
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

void findEdges(cv::Mat edgeBoundary, cv::Point scanStart, cv::Point scanEnd, cv::Mat& scanROI, cv::Mat& edges, double heightThresh, int order = 1) {

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
		cv::threshold(scanROI, heightMask, heightThresh, 255, cv::THRESH_BINARY);
		heightMask.convertTo(heightMask, CV_8U);
		//cv::normalize(heightMask, heightMask, 0, 255, cv::NORM_MINMAX, CV_8U);

		// Search within the edges of the dialated raster for the actual edges
		cv::Range searchRange;
		// Take derivative and blur
		cv::Mat dx, ROIblur;
		int aperture_size = 7;
		int sigma = 11;
		int sz = 19;
		cv::GaussianBlur(scanROI, ROIblur, cv::Size(sz, sz), (double)sigma / 10);
		cv::Sobel(ROIblur, dx, -1, order, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);

		//plotScan(scanROI);

		int foundEdges[2] = { NAN };
		int maxIdx[2] = { NAN };
		int minIdx[2] = { NAN };
		// loop through all the search windows
		for (auto it = windowPts.begin(); it != windowPts.end(); std::advance(it, 2)) {
			// Check if the search window is at least 0.5mm wide
			if ((*std::next(it) - *it) > MM2PIX(0.5)) {
				if (order == 1) {
					// set the window search range
					searchRange = cv::Range(*it, *std::next(it));
					//find the edges by finding the local extrema of the profile derivative 
					cv::minMaxIdx(dx(cv::Range::all(), searchRange), NULL, NULL, minIdx, maxIdx);
					foundEdges[0] = maxIdx[1] + searchRange.start;
					foundEdges[1] = minIdx[1] + searchRange.start;
				}
				else if (order == 2) {
					// split the window at the minimum point
					searchRange = cv::Range(*it, *std::next(it));
					cv::minMaxIdx(dx(cv::Range::all(), searchRange), NULL, NULL, minIdx, NULL);
					// search the first half of the window
					searchRange = cv::Range(*it, minIdx[1] + *it);
					cv::minMaxIdx(dx(cv::Range::all(), searchRange), NULL, NULL, NULL, maxIdx);
					foundEdges[0] = maxIdx[1] + searchRange.start;
					// search the second half of the window
					searchRange = cv::Range(minIdx[1] + *it, *std::next(it));
					cv::minMaxIdx(dx(cv::Range::all(), searchRange), NULL, NULL, NULL, maxIdx);
					foundEdges[1] = maxIdx[1] + searchRange.start;
				}

				// mark edges on local profile and global ROI
				slope = cv::Point2d(scanEnd - scanStart) / lineit.count;
				for (int j = 0; j < 2; j++) {
					edgeCoord = cv::Point2d(scanStart) + foundEdges[j] * slope;
					// check if edges are within height mask
					if ((heightMask.at<uchar>(cv::Point(foundEdges[j], 0)) == 255) && (foundEdges[j] != *it) && (foundEdges[j] != *std::next(it))) {
						//locEdges.at<uchar>(cv::Point(foundEdges[j], 0)) = 255;
						edges.at<uchar>(cv::Point2i(edgeCoord)) = 255;
					}
				}
				// mark window borders
				//locWin.at<uchar>(cv::Point(searchRange.start, 0)) = 255;
				//locWin.at<uchar>(cv::Point(searchRange.end, 0)) = 255;
			}
		}
	}

	return;
}

void findEdges2(cv::Mat edgeBoundary, cv::Point scanStart, cv::Point scanEnd, cv::Mat& scanROI, cv::Mat& edges) {

	if ((scanStart != cv::Point(-1, -1)) && (scanEnd != cv::Point(-1, -1))) //Check if scan is within ROI
	{
		cv::LineIterator lineit(edgeBoundary, scanStart, scanEnd, 8);
		cv::Point2d edgeCoord, slope;

		cv::Mat ROIblur, morph, pkMask, mask, median, baseline, kern;
		cv::Scalar mean, stddev;
		int sigma, sz;

		// Blur the scan
		sigma = 61;
		sz = 9;
		cv::GaussianBlur(scanROI, ROIblur, cv::Size(sz, sz), (double)sigma / 10);


		cv::meanStdDev(ROIblur, mean, stddev);
		if (stddev.val[0] > 0.015){
			// find the meadian height value of the scan and use the median mask to average only the heights below the median
			cv::normalize(ROIblur, median, 0, 255, cv::NORM_MINMAX, CV_8U);
			cv::threshold(median, median, 2, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU);
			median.convertTo(median, CV_8U);
			kern = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
			cv::morphologyEx(median, median, cv::MORPH_CLOSE, kern, cv::Point(-1, -1), 1);
			cv::meanStdDev(scanROI, mean, stddev, median);

			// truncate all values above the mean baseline value and set as the baseline then smooth the baseline
			cv::threshold(ROIblur, baseline, mean.val[0], 255, cv::THRESH_TRUNC);
			cv::morphologyEx(baseline, baseline, cv::MORPH_OPEN, kern, cv::Point(-1, -1), 2);
			cv::blur(baseline, baseline, cv::Size(scanROI.cols / 8, scanROI.cols / 8));

			// merge the baseline and the scan
			ROIblur.copyTo(morph);
			baseline.copyTo(morph, median);
			kern = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
			cv::morphologyEx(morph, morph, cv::MORPH_CLOSE, kern, cv::Point(-1, -1), 2);
			sigma = 19;
			cv::GaussianBlur(morph, morph, cv::Size(sz, sz), (double)sigma / 10);

			// offset the baseline
			baseline += 0.004;
			// find the parts of the scan that are above the baseline
			cv::compare(morph, baseline, pkMask, cv::CMP_GT);
			cv::Sobel(pkMask, pkMask, -1, 2, 0, 3, 1, 0, cv::BORDER_REPLICATE);

			// filter out any peaks near the edges of the scan
			mask = cv::Mat::zeros(pkMask.size(), CV_8U);
			mask(cv::Range(0, 1), cv::Range(MM2PIX(0.5), mask.cols - MM2PIX(0.5))) = 255;
			cv::bitwise_and(pkMask, mask, pkMask);
			
			std::vector<cv::Point> peaks;
			cv::findNonZero(pkMask, peaks);

			// debugging edge detection
			//auto ax1 = CvPlot::makePlotAxes();
			//ax1.create<CvPlot::Series>(scanROI, "-k");
			//ax1.create<CvPlot::Series>(ROIblur, "-b");
			//ax1.create<CvPlot::Series>(morph, "-m");
			//ax1.create<CvPlot::Series>(baseline, "-c");
			//cv::Mat ax_1 = ax1.render();

			// mark edges on global ROI
			slope = cv::Point2d(scanEnd - scanStart) / lineit.count;
			for (auto it = peaks.begin(); it != peaks.end(); ++it) {
				edgeCoord = cv::Point2d(scanStart) + (*it).x * slope;
				edges.at<uchar>(cv::Point2i(edgeCoord)) = 255;
			}
			cv::bitwise_and(edgeBoundary, edges, edges);
		}
	}
}