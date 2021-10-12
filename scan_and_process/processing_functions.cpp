#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <iterator>

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define CVPLOT_HEADER_ONLY 
#include <CvPlot/cvplot.h>


void getScan(double data[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& scan) {
	int fbIdx[2];
	int scanStartIdx;
	cv::Mat scanVoltage_8U, scanEdges, scanEdgesIdx;

	cv::Mat dataMat(NUM_PROFILE_PTS, NUM_DATA_SAMPLES, CV_64F, data); // copying the collected data into a matrix
	
	// Get the position feedback when the laser was triggered
	cv::minMaxIdx(dataMat.row(1), NULL, NULL, fbIdx, NULL); //find the rising edge of the trigger signal sent to the laser
	fbk->x = dataMat.at<double>(2, fbIdx[1]); // assigning the position feedback values
	fbk->y = dataMat.at<double>(3, fbIdx[1]);
	fbk->z = dataMat.at<double>(4, fbIdx[1]);
	fbk->T = dataMat.at<double>(5, fbIdx[1]);

	// Finding the voltage header of the scanner signal, i.e find the start of the scanned profile
	// Search only the data after the triggering signal was sent (after index fbIdx[1])
	cv::normalize(dataMat(cv::Range(0, 1), cv::Range(fbIdx[1], dataMat.cols)), scanVoltage_8U, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::Canny(scanVoltage_8U, scanEdges, 10, 30, 3);
	cv::findNonZero(scanEdges, scanEdgesIdx);

	scanStartIdx = scanEdgesIdx.at<int>(1, 0) + fbIdx[1]; // Starting index of the scan profile
	scan = dataMat(cv::Rect(scanStartIdx, 0, NUM_PROFILE_PTS, 1)).clone() / OPAMP_GAIN; // Isolating the scanned profile and converting to height

	return;
}

//=============================================

void scan2ROI(cv::Mat& scan, const Coords fbk, const std::vector<double>& printROI, cv::Size rasterSize, cv::Mat& scanROI, cv::Point &scanStart, cv::Point& scanEnd) {

	std::vector<double> XY_start(2),XY_end(2);
	double X, Y;
	double R = SCAN_OFFSET;
	double local_x;
	int startIdx = -1, endIdx = 0;

	for (int i = 0; i < scan.cols; i++) {
		// Local coordinate of the scanned point
		local_x = -SCAN_WIDTH / 2 + i * SCAN_WIDTH / (int(scan.cols) - 1);
		// Transforming local coordinate to global coordinate
		X = fbk.x - R * cos(fbk.T * PI / 180) - local_x * sin(fbk.T * PI / 180);
		Y = fbk.y - R * sin(fbk.T * PI / 180) + local_x * cos(fbk.T * PI / 180);
		// Check if scanned point in outside the print ROI 
		if ((X < printROI[0] || printROI[2] < X || Y < printROI[1] || printROI[3] < Y)) {
		}
		else if (startIdx == -1) {
			startIdx = i;
			XY_start = { X, Y };
		}
		else { 
			endIdx = i; 
			XY_end = { X, Y };
		}
	}
	//convert the start and end (X,Y) coordinates of the scan to points on the image
	cv::Point startPx(std::round((XY_start[1] - printROI[1]) / PIX2MM), std::round((XY_start[0] - printROI[0]) / PIX2MM));
	cv::Point endPx(std::round((XY_end[1] - printROI[1]) / PIX2MM), std::round((XY_end[0] - printROI[0]) / PIX2MM));

	scanStart = cv::Point (std::round((XY_start[1] - printROI[1]) / PIX2MM), std::round((XY_start[0] - printROI[0]) / PIX2MM));
	scanEnd = cv::Point (std::round((XY_end[1] - printROI[1]) / PIX2MM), std::round((XY_end[0] - printROI[0]) / PIX2MM));
	cv::Range scanROIRange = cv::Range(startIdx, endIdx);

	// Interpolate scan so it is the same scale as the raster reference image
	cv::LineIterator it(rasterSize, scanStart, scanEnd, 8); // make a line iterator between the start and end points of the scan
	cv::resize(scan.colRange(scanROIRange), scanROI, cv::Size(it.count, scan.rows), cv::INTER_LINEAR);

	return;
}

//=============================================

void findEdges(cv::Mat edgeBoundary, cv::Point scanStart, cv::Point scanEnd, cv::Mat& scanROI, cv::Mat& gblEdges, cv::Mat& locEdges, double heightThresh) {

	cv::LineIterator lineit(edgeBoundary, scanStart, scanEnd, 8);
	std::vector<int> windowPts;
	windowPts.reserve(lineit.count);
	std::vector<int> windowCenters;
	windowCenters.reserve(lineit.count);
	uchar lastVal = 0;
	uchar curVal = 0;

	int maxIdx[2];
	int minIdx[2];
	cv::Mat locCent(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	cv::Mat searchWins(scanROI.size(), CV_8U, cv::Scalar({ 0 }));
	

	// find the intersection of the scan and the edge boundary using a line iterator
	for (int i = 0; i < lineit.count; i++, ++lineit) {
		curVal = *(const uchar*)*lineit;
		if ((curVal == 255) && (lastVal == 0) && (i != 0)) { // find rising edges
			windowPts.push_back(lineit.pos().x);
		}
		if ((curVal == 0) && (lastVal == 255) && (i != 0)) { // find falling edges
			windowPts.push_back(lineit.pos().x);
		}
		lastVal = curVal;
	}
	if ((windowPts.size() % 2) != 0) {
		std::cout << "ERROR: odd number of edges" << std::endl;
		return ;
	}

	// find the intersections of the scanner and the raster pattern
	for (int i = 0; i < lineit.count; i++, ++lineit) {
		if (*(const uchar*)*lineit == 255) {
			windowCenters.push_back(lineit.pos().x);
		}
	}


	// create a height mask for the scan profile to remove all edges below a height threshold
	cv::Mat heightMask;
	cv::threshold(scanROI, heightMask, heightThresh, 1, cv::THRESH_BINARY);
	cv::normalize(heightMask, heightMask, 0, 255, cv::NORM_MINMAX, CV_8U);


	// Search within the edges of the dialated raster for the actual edges
	cv::Range searchRange;
	cv::Mat searchWindow;
	cv::Mat edges;
	std::vector<cv::Point> edgeCoords;

	cv::Mat dx, scanROIblur;
	int aperture_size = 7;
	int sigma = 61;
	int sz = 19;
	cv::GaussianBlur(scanROI, scanROIblur, cv::Size(sz, sz), (double)sigma / 10);
	cv::Sobel(scanROIblur, dx, -1, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
	int foundEdges[2];

	// loop through all the search windows
	for (auto it = windowCenters.begin(); it != windowCenters.end(); ++it) {
		//std::cout << *it << std::endl;
		searchRange = cv::Range(*it - _MM2PIX(SRCH_WND_WDTH / 2), *it + _MM2PIX(SRCH_WND_WDTH / 2));
		std::cout << "search range = " << searchRange << std::endl;
		cv::minMaxIdx(dx(cv::Range::all(), searchRange), NULL, NULL, minIdx, maxIdx); 
		foundEdges[0] = maxIdx[1] + searchRange.start;
		foundEdges[1] = minIdx[1] + searchRange.start;
		//std::cout << "Edges = " << maxIdx[1]+searchRange.start << " & " << minIdx[1] + searchRange.start << std::endl;
		// mark edges on local profile and global ROI
		for (int j = 0; j < 2; j++) {
			if (heightMask.at<uchar>(cv::Point(foundEdges[j], 0)) == 255) { // check if edges are within height mask
				locEdges.at<uchar>(cv::Point(foundEdges[j] , 0)) = 255;
				gblEdges.at<uchar>(cv::Point(foundEdges[j] + scanStart.x, scanStart.y)) = 255;
			}
		}
		searchWins.at<uchar>(cv::Point(searchRange.start, 0)) = 255;
		searchWins.at<uchar>(cv::Point(searchRange.end, 0)) = 255;
	}

	// modified old loop
	//for (auto it = windowPts.begin(); it != windowPts.end(); std::advance(it, 2)) {

	//	searchRange = cv::Range(*it , *std::next(it) );
	//	std::cout << "search range = " << searchRange << std::endl;
	//	cv::minMaxIdx(dx(cv::Range::all(), searchRange), NULL, NULL, minIdx, maxIdx);
	//	foundEdges[0] = maxIdx[1] + searchRange.start;
	//	foundEdges[1] = minIdx[1] + searchRange.start;
	//	//std::cout << "Edges = " << maxIdx[1]+searchRange.start << " & " << minIdx[1] + searchRange.start << std::endl;
	//	// mark edges on local profile and global ROI
	//	for (int j = 0; j < 2; j++) {
	//		//if (heightMask.at<uchar>(cv::Point(foundEdges[j], 0)) == 255) { // check if edges are within height mask
	//		locEdges.at<uchar>(cv::Point(foundEdges[j], 0)) = 255;
	//		gblEdges.at<uchar>(cv::Point(foundEdges[j] + scanStart.x, scanStart.y)) = 255;
	//		//}
	//	}
	//	searchWins.at<uchar>(cv::Point(searchRange.start, 0)) = 255;
	//	searchWins.at<uchar>(cv::Point(searchRange.end, 0)) = 255;
	//}


	//Plotting
	cv::Mat pltEdges(scanROI.size(), CV_32F, cv::Scalar(NAN));
	cv::Mat pltWins(scanROI.size(), CV_32F, cv::Scalar(NAN));
	// need to repeat setting the matrix to Nan because they get change to 0 after first copyTo
	dx.copyTo(pltEdges, locEdges);
	dx.copyTo(pltWins, searchWins);
	pltEdges = cv::Scalar(NAN);
	pltWins = cv::Scalar(NAN);
	dx.copyTo(pltEdges, locEdges);
	dx.copyTo(pltWins, searchWins);

	cv::namedWindow("derivative", cv::WINDOW_NORMAL);
	auto axes_dx = CvPlot::makePlotAxes();
	axes_dx.create<CvPlot::Series>(dx, "-k");
	axes_dx.create<CvPlot::Series>(pltWins, "bo");
	axes_dx.create<CvPlot::Series>(pltEdges, "ro");
	CvPlot::show("derivative", axes_dx);

	pltEdges = cv::Scalar(NAN);
	pltWins = cv::Scalar(NAN);
	scanROI.copyTo(pltEdges, locEdges);
	scanROI.copyTo(pltWins, searchWins);
	cv::namedWindow("profile", cv::WINDOW_NORMAL);
	auto axes_prfl = CvPlot::makePlotAxes();
	axes_prfl.create<CvPlot::Series>(scanROI, "-k");
	axes_prfl.create<CvPlot::Series>(pltWins, "bo");
	axes_prfl.create<CvPlot::Series>(pltEdges, "ro");
	CvPlot::show("profile", axes_prfl);





	//// displaying the edges
	//cv::Mat scanGray;
	//cv::normalize(scanROI, scanGray, 0, 255, cv::NORM_MINMAX, CV_8U);

	//cv::Mat m(scanGray.size(), CV_8UC3, cv::Scalar({ 255, 0, 255, 0 }));
	//cv::Mat c(scanGray.size(), CV_8UC3, cv::Scalar({ 255, 255, 0, 0 }));
	//cv::Mat b(scanGray.size(), CV_8UC3, cv::Scalar({ 255, 0, 0, 0 }));
	//cv::cvtColor(scanGray, scanGray, cv::COLOR_GRAY2BGR);
	//m.copyTo(scanGray, locEdges);
	//c.copyTo(scanGray, locCent);
	//b.copyTo(scanGray, searchWins);
	//cv::namedWindow("local", cv::WINDOW_NORMAL);
	//cv::imshow("local", scanGray);
	//cv::waitKey(1);
}