#include <iostream>
#include <vector>
#include <iterator> 
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "myGlobals.h"
#include "constants.h"
#include "gaussianSmooth.h"

/**
 * @brief Returns a vector of regions around each rod of a raster pattern.
 * @param rasterCoords[in] Vector of x,y points of the raster pattern
 * @param width[in] width of the regions
 * @param edgeRegions[out] Vector of regions that are around each rod
*/
void makeEdgeRegions(const std::vector<cv::Point>& rasterCoords, double width, std::vector<cv::Rect>& edgeRegions) {
	
	int pixWidth = MM2PIX(width);
	// Rods
	for (auto it = rasterCoords.begin(); it != rasterCoords.end(); std::advance(it,2)) {
		edgeRegions.push_back(cv::Rect(*it - cv::Point(pixWidth / 2, 0), *std::next(it, 1) + cv::Point(pixWidth / 2, 0)));
	}
	// Corners
	// TODO: add corner regions
}

/**
 * @brief Finds the left and right edges of the material and smooths them
 * @param edgeRegions[in] Rectangle specifying the region of the image to search for the edges 
 * @param gblEdges[in] Image of edge points found by the scanner
 * @param lEdgePts[out] Filtered points that make up the edge in the left half of the ROI
 * @param rEdgePts[out] Filtered points that make up the edge in the right half of the ROI
 * @param interp[in] Flag indicating whether to interpolate the output edge points 
*/
void getMatlEdges(const cv::Rect& edgeRegions, cv::Mat& gblEdges, std::vector<std::vector<cv::Point>>& lEdgePts, std::vector<std::vector<cv::Point>>& rEdgePts, bool interp = false) {
	std::vector<cv::Point> unfiltLeft, unfiltRight, filtPts;
	cv::Mat interpPts;
	// find the left and right edge points in the regions
	cv::Rect lRegion = edgeRegions - cv::Size(edgeRegions.width / 2, 0);
	cv::Rect rRegion = lRegion + cv::Point(edgeRegions.width / 2, 0);
	// find the edges in the search regions
	cv::findNonZero(gblEdges(lRegion), unfiltLeft);
	cv::findNonZero(gblEdges(rRegion), unfiltRight);
	// converting points back to global coordinates
	for (auto it = unfiltLeft.begin(); it != unfiltLeft.end(); ++it) {*it += lRegion.tl();}
	for (auto it = unfiltRight.begin(); it != unfiltRight.end(); ++it) {*it += rRegion.tl();}
	//TODO: Sort the edges

	// Smoothing the edges
	if(interp){ interpPts = cv::Mat(gblEdges.size(), CV_8UC1, cv::Scalar(0)); }
	// Left edge
	gaussianSmoothX(unfiltLeft, filtPts, 7, 3);
	if (interp) {
		cv::polylines(interpPts, filtPts, false, cv::Scalar(255), 1);
		cv::findNonZero(interpPts, filtPts);
	}
	lEdgePts.push_back(filtPts);
	// Right edge
	gaussianSmoothX(unfiltRight, filtPts, 7, 3);
	if (interp) {
		interpPts = cv::Scalar(0); // remove the left edge points
		cv::polylines(interpPts, filtPts, false, cv::Scalar(255), 1);
		cv::findNonZero(interpPts, filtPts);
	}
	rEdgePts.push_back(filtPts);

}

/**
 * @brief Calculates the material centerline and width errors.
 * @param[in,out] centerline Vector of points that make up the desired centerline. Input as 2 coordinate pairs; output as interpolated coordinate pairs  
 * @param[in] width Desired width of the material in mm
 * @param[in] rasterSize Size of the raster image
 * @param[in] lEdgePts Points making up the left edge of the rod
 * @param[in] rEdgePts Points making up the right edge of the rod
 * @param[out] errCL Material centerline error
 * @param[out] errWD Material width error
*/
void getMatlErrors(std::vector<cv::Point>& centerline, double width, cv::Size rasterSize, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts, std::vector <std::vector<double>>& errCL, std::vector<std::vector<double>>& errWD) {
	//TODO: Remove rasterSize as an imput if raster is a global variable
	// Calculate Errors
	std::vector<double> ecl, ewd; // centerline and width errors
	cv::Mat lEdge = cv::Mat(rasterSize, CV_8UC1, cv::Scalar(255)); // Image to draw the left edge on
	cv::Mat rEdge = cv::Mat(rasterSize, CV_8UC1, cv::Scalar(255)); // Image to draw the right edge on
	cv::LineIterator lnit(lEdge, centerline.front(), centerline.back(), 8);
	centerline.clear(); // clear the points in the centerline vector
	//TODO: all error points are stored, not just for a single rod
	ecl.reserve(lnit.count);
	ewd.reserve(lnit.count);
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
		ecl.push_back((rEdge.at<float>(lnit.pos()) - lEdge.at<float>(lnit.pos())) / 2);
		ewd.push_back(lEdge.at<float>(lnit.pos()) + rEdge.at<float>(lnit.pos()) - width);
	}
	errCL.push_back(ecl);
	errWD.push_back(ewd);
}