#include "errors.h"
#include <iostream>
#include <vector>
#include <iterator> 
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "myGlobals.h"
#include "constants.h"
#include "gaussianSmooth.h"

struct sortX {
	bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x); }
}; 

struct sortDist {
	sortDist(cv::Point pt0) { this->pt0 = pt0; }
	bool operator() (cv::Point pt1, cv::Point pt2) { return (cv::norm(pt0-pt1) < cv::norm(pt0 - pt2)); }
	cv::Point pt0;
};

void getMatlEdges(const cv::Rect& segmentROI, const cv::Mat& gblEdges, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts) {
	std::vector<cv::Point> unfiltLeft, unfiltRight;
	cv::Mat edgePts = cv::Mat(gblEdges.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat mask = cv::Mat(gblEdges.size(), CV_8UC1, cv::Scalar(0));

	// find the left and right edge points in the regions
	cv::Rect lRegion = segmentROI - cv::Size(0, segmentROI.height / 2);
	cv::Rect rRegion = lRegion + cv::Point(0, segmentROI.height / 2);

	// find the edges in the search regions
	cv::findNonZero(gblEdges(lRegion), unfiltLeft);
	cv::findNonZero(gblEdges(rRegion), unfiltRight);
	// converting points back to global coordinates
	for (auto it = unfiltLeft.begin(); it != unfiltLeft.end(); ++it) {*it += lRegion.tl();}
	for (auto it = unfiltRight.begin(); it != unfiltRight.end(); ++it) {*it += rRegion.tl();}
	// Sort the edges
	std::sort(unfiltLeft.begin(), unfiltLeft.end(), sortX());
	std::sort(unfiltRight.begin(), unfiltRight.end(), sortX());

	// Smoothing the edges
	
	// Left edge
	// smooth out the raw points
	gaussianSmoothX(unfiltLeft, lEdgePts, 7, 2); // 7, 3
	// draw smoothed points as a line and then dialate the line to form a mask
	cv::polylines(mask, lEdgePts, false, cv::Scalar(255), 1);
	cv::morphologyEx(mask, mask, cv::MORPH_DILATE, cv::Mat::ones(5, 5, CV_8UC1), cv::Point(-1, -1), 1);
	// copy raw points within the mask to remove outliers
	gblEdges.copyTo(edgePts, mask);
	cv::findNonZero(edgePts, unfiltLeft);
	// smooth the points
	std::sort(unfiltLeft.begin(), unfiltLeft.end(), sortX());
	gaussianSmoothX(unfiltLeft, lEdgePts, 3, 1);

	// reset all images
	edgePts = cv::Scalar(0);
	mask = cv::Scalar(0);
	
	// Right edge
	gaussianSmoothX(unfiltRight, rEdgePts, 7, 2);
	cv::polylines(mask, rEdgePts, false, cv::Scalar(255), 1);
	cv::morphologyEx(mask, mask, cv::MORPH_DILATE, cv::Mat::ones(5,5,CV_8UC1), cv::Point(-1, -1), 1);
	gblEdges.copyTo(edgePts, mask);
	cv::findNonZero(edgePts, unfiltRight);
	std::sort(unfiltRight.begin(), unfiltRight.end(), sortX());
	gaussianSmoothX(unfiltRight, rEdgePts, 3, 1);
}

void getMatlErrors(std::vector<cv::Point>& centerline, double width, cv::Size rasterSize, const std::vector<cv::Point>& lEdgePts, const std::vector<cv::Point>& rEdgePts, std::vector<double>& errCL, std::vector<double>& errWD) {
	cv::Mat lEdge = cv::Mat(rasterSize, CV_8UC1, cv::Scalar(255)); // Image to draw the left edge on
	cv::Mat rEdge = cv::Mat(rasterSize, CV_8UC1, cv::Scalar(255)); // Image to draw the right edge on
	cv::LineIterator lnit(lEdge, centerline.front(), centerline.back(), 8);
	centerline.clear(); // clear the points in the centerline vector
	errCL.clear(); // clear the errors
	errWD.clear();
	errCL.reserve(lnit.count);
	errWD.reserve(lnit.count);
	// See how far the edges are from the unmodified path
	// Draw the smoothed edges on an image
	cv::polylines(lEdge, lEdgePts, false, cv::Scalar(0), 1);
	cv::polylines(rEdge, rEdgePts, false, cv::Scalar(0), 1);
	// apply a distance transform to the image with the smoothed edges
	cv::distanceTransform(lEdge, lEdge, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F); //NOTE: regarding speed -  DIST_MASK_5 (3.9ms/rod) < DIST_MASK_PRECISE (4.7ms/rod) < DIST_MASK_3 (9.1ms/rod) 
	cv::distanceTransform(rEdge, rEdge, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
	// iterate over the target centerline to calculate the errors
	for (int i = 0; i < lnit.count; i++, ++lnit) {
		//TODO: check if there is a measured edge to the left / right of the path
		// maybe check if the dXform distance is greater than the boundary width
		centerline.push_back(lnit.pos()); // add the points from the centerline
		errCL.push_back((PIX2MM(static_cast<__int64>(rEdge.at<float>(lnit.pos())) - static_cast<__int64>(lEdge.at<float>(lnit.pos())))) / 2);
		errWD.push_back(PIX2MM(static_cast<__int64>(lEdge.at<float>(lnit.pos())) + static_cast<__int64>(width - rEdge.at<float>(lnit.pos()))));
	}
}

void getErrorsAt(std::vector<cv::Point>& waypoints, std::vector<double>targetWidths, cv::Size rasterSize, const std::vector<cv::Point>& lEdgePts, const std::vector<cv::Point>& rEdgePts, std::vector<double>& errCL, std::vector<double>& errWD) {
	cv::Mat lEdge = cv::Mat(rasterSize, CV_8UC1, cv::Scalar(255)); // Image to draw the left edge on
	cv::Mat rEdge = cv::Mat(rasterSize, CV_8UC1, cv::Scalar(255)); // Image to draw the right edge on
	errCL.clear(); // clear the errors
	errWD.clear();
	errCL.reserve(waypoints.size());
	errWD.reserve(waypoints.size());
	int i = 0;

	// Create rectangle containing area with material on both sides of the rater
	int minX = (std::max)(lEdgePts.front().x, rEdgePts.front().x);
	int maxX = (std::min)(lEdgePts.back().x, rEdgePts.back().x);
	int minY = (*std::min_element(lEdgePts.begin(), lEdgePts.end(), [](const cv::Point& pt1, const cv::Point& pt2) {return pt1.y < pt2.y; })).y;
	int maxY = (*std::max_element(rEdgePts.begin(), rEdgePts.end(), [](const cv::Point& pt1, const cv::Point& pt2) {return pt1.y < pt2.y; })).y;
	cv::Rect edgeRoi = cv::Rect(minX, minY, maxX-minX, maxY-minY);
	// create a contour from the edges //UNUSED
	//std::vector<cv::Point> edgeContour;
	//edgeContour.reserve(lEdgePts.size() + rEdgePts.size()); // preallocate memory
	//edgeContour.insert(edgeContour.end(), lEdgePts.begin(), lEdgePts.end());
	//edgeContour.insert(edgeContour.end(), rEdgePts.rbegin(), rEdgePts.rend());
	
	// See how far the edges are from the unmodified path
	// Draw the smoothed edges on an image
	cv::polylines(lEdge, lEdgePts, false, cv::Scalar(0), 1);
	cv::polylines(rEdge, rEdgePts, false, cv::Scalar(0), 1);
	// apply a distance transform to the image with the smoothed edges
	cv::distanceTransform(lEdge, lEdge, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F); //NOTE: regarding speed -  DIST_MASK_5 (3.9ms/rod) < DIST_MASK_PRECISE (4.7ms/rod) < DIST_MASK_3 (9.1ms/rod) 
	cv::distanceTransform(rEdge, rEdge, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
	// iterate over the waypoints to calculate the errors
	for (auto it = waypoints.begin(); it != waypoints.end(); ++it, i++) {
		// Check if the waypoint is within the countour created by the edges
		//if (cv::pointPolygonTest(edgeContour, *it, false) >= 0) { // UNUSED
		if (edgeRoi.contains(*it)) {
			errCL.push_back((PIX2MM(static_cast<__int64>(rEdge.at<float>(*it)) - static_cast<__int64>(lEdge.at<float>(*it)))) / 2);
			errWD.push_back(targetWidths[i] - PIX2MM(static_cast<__int64>(lEdge.at<float>(*it)) + static_cast<__int64>(rEdge.at<float>(*it))));
		}
		else {
			// HACK: set invalid errors to NAN
			errCL.push_back(NAN);
			errWD.push_back(NAN);
		}
	}
}