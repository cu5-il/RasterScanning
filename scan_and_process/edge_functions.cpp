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
 * @brief Breaks up the raster pattern into segments. Each vertical rod in the raster is a segment; corners are neglected.
 * @param[in] rasterCoords Vector of (x,y) points of the raster pattern
 * @param[in] ROIwidth of the ROI
 * @param[out] ROIs Vector of regions that are around each rod. Used to determine which edges belong to which segment
 * @param[out] centerlines Vector of point pairs defining the centerline of the region
 * @param[out] scanDonePts Point in the raster pattern when the region has been finished scanning. Set to midpoint of the following rod except for the last region, which is just the offset of the last point of the raster
*/
void makeSegments(const std::vector<cv::Point>& rasterCoords, double ROIwidth, std::vector<cv::Rect>& ROIs, std::vector<std::vector<cv::Point>>& centerlines, std::vector<cv::Point>& scanDonePts) {
	int pixWidth = MM2PIX(ROIwidth);
	// Rods
	for (auto it = rasterCoords.begin(); it != rasterCoords.end(); std::advance(it,2)) {
		ROIs.push_back(cv::Rect(*it - cv::Point(pixWidth / 2, 0), *std::next(it, 1) + cv::Point(pixWidth / 2, 0)));
		centerlines.push_back(std::vector<cv::Point> { *it, * std::next(it, 1)});
	}
	// defining the point when the region has been completely scanned as the midpoint of the next centerline
	for (auto it = std::next(centerlines.begin()); it != centerlines.end(); ++it) {
		scanDonePts.push_back((*(*it).begin() + *(*it).end()) / 2);
	}
	// TODO: change scanning termination point to last point in raster + scanner offset
	// Make the point to end scanning for the final region the last point in the raster
	scanDonePts.push_back(*rasterCoords.end());

	// Corners
	// TODO: add corner regions
}

/**
 * @brief Finds the left and right edges of the material and smooths them
 * @param[in] edgeRegions Rectangle specifying the region of the image to search for the edges 
 * @param[in] gblEdges Image of edge points found by the scanner
 * @param[out] lEdgePts Filtered points that make up the edge in the left half of the ROI
 * @param[out] rEdgePts Filtered points that make up the edge in the right half of the ROI
 * @param[in] interp Flag indicating whether to interpolate the output edge points 
*/
void getMatlEdges(const cv::Rect& edgeRegions, cv::Mat& gblEdges, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts, bool interp = false) {
	std::vector<cv::Point> unfiltLeft, unfiltRight;
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
	gaussianSmoothX(unfiltLeft, lEdgePts, 7, 3);
	if (interp) {
		cv::polylines(interpPts, lEdgePts, false, cv::Scalar(255), 1);
		cv::findNonZero(interpPts, lEdgePts);
	}
	// Right edge
	gaussianSmoothX(unfiltRight, rEdgePts, 7, 3);
	if (interp) {
		interpPts = cv::Scalar(0); // remove the left edge points
		cv::polylines(interpPts, rEdgePts, false, cv::Scalar(255), 1);
		cv::findNonZero(interpPts, rEdgePts);
	}
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
	//TODO: Remove rasterSize as an input if raster is a global variable
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