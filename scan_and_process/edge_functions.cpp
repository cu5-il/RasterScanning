#include <iostream>
#include <vector>
#include <iterator> 
#include <algorithm>
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

/**
 * @brief Breaks up the raster pattern into segments. Each vertical rod in the raster is a segment; corners are neglected.
 * @param[in] rasterCoords Vector of (x,y) points of the raster pattern
 * @param[in] ROIwidth of the ROI
 * @param[out] ROIs Vector of regions that are around each rod. Used to determine which edges belong to which segment
 * @param[out] centerlines Vector of point pairs defining the centerline of the region
 * @param[out] scanDonePts Point in the raster pattern when the region has been finished scanning. Set to midpoint of the following rod except for the last region, which is just the offset of the last point of the raster
*/
void makeSegments(const std::vector<cv::Point>& rasterCoords, double ROIwidth, std::vector<Segment>& seg, cv::Point2d initPos) {
	std::vector<cv::Rect> ROIs;
	std::vector<std::vector<cv::Point>> centerlines;
	std::vector<cv::Point2d> scanDonePts;
	int pixWidth = MM2PIX(ROIwidth);
	int direction = 1;
	// Rods
	for (auto it = rasterCoords.begin(); it != rasterCoords.end(); std::advance(it,2)) {
		ROIs.push_back(cv::Rect(*it - cv::Point(0, pixWidth / 2), *std::next(it, 1) + cv::Point(0, pixWidth / 2)));
		centerlines.push_back(std::vector<cv::Point> { *it, * std::next(it, 1)});
	}
	// defining the point when the region has been completely scanned as the midpoint of the next centerline
	for (auto it = std::next(centerlines.begin()); it != centerlines.end(); ++it) {
		scanDonePts.push_back(PIX2MM((cv::Point2d( (*it).front() + (*it).back()) / 2  - cv::Point2d(rasterCoords.front()) )) + initPos);
	}
	// TODO: change scanning termination point to last point in raster + scanner offset
	// Make the point to end scanning for the final region the last point in the raster
	scanDonePts.push_back(PIX2MM((cv::Point2d(rasterCoords.back()) - cv::Point2d(rasterCoords.front()) )) + initPos);

	// Placing all of the values in the Segment class
	if ((ROIs.size() == centerlines.size()) && (ROIs.size() == scanDonePts.size())) {
		for (int i = 0; i < ROIs.size(); i++){
			seg.push_back(Segment(ROIs[i], centerlines[i], scanDonePts[i],direction));
			direction *= -1; // flip direction
		}
	}

	// Corners
	// TODO: add corner regions
}

/**
 * @brief Finds the left and right edges of the material and smooths them
 * @param[in] segmentROI Rectangle specifying the region of the image to search for the edges 
 * @param[in] gblEdges Image of edge points found by the scanner
 * @param[out] lEdgePts Filtered points that make up the edge in the left half of the ROI
 * @param[out] rEdgePts Filtered points that make up the edge in the right half of the ROI
 * @param[in] interp Flag indicating whether to interpolate the output edge points 
*/
void getMatlEdges(const cv::Rect& segmentROI, const cv::Mat& gblEdges, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts, bool interp = false) {
	std::vector<cv::Point> unfiltLeft, unfiltRight;
	cv::Mat interpPts;
	// find the left and right edge points in the regions
	//cv::Rect lRegion = segmentROI - cv::Size(segmentROI.width / 2, 0);
	//cv::Rect rRegion = lRegion + cv::Point(segmentROI.width / 2, 0);
	cv::Rect lRegion = segmentROI - cv::Size(0, segmentROI.height / 2);
	cv::Rect rRegion = lRegion + cv::Point(0, segmentROI.height / 2);

	cv::Mat temp = cv::Mat::zeros(gblEdges.size(), CV_8UC3);
	cv::rectangle(temp, segmentROI, cv::Scalar(255, 0, 0), 8);
	cv::rectangle(temp, lRegion, cv::Scalar(0,255, 0), 4);
	cv::rectangle(temp, rRegion, cv::Scalar(0, 0, 255), 4);
	// find the edges in the search regions
	cv::findNonZero(gblEdges(lRegion), unfiltLeft);
	cv::findNonZero(gblEdges(rRegion), unfiltRight);
	// converting points back to global coordinates
	for (auto it = unfiltLeft.begin(); it != unfiltLeft.end(); ++it) {*it += lRegion.tl();}
	for (auto it = unfiltRight.begin(); it != unfiltRight.end(); ++it) {*it += rRegion.tl();}
	// Sort the edges
	std::sort(unfiltLeft.begin(), unfiltLeft.end(), sortX());
	std::sort(unfiltRight.begin(), unfiltRight.end(), sortX());
	//std::sort(unfiltLeft.begin(), unfiltLeft.end(), sortDist(cv::Point(0,0))); // TODO: change (0,0) to segment.centerline()[0]?
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
 * @param[out] errCL Material centerline error in pixels
 * @param[out] errWD Material width error in pixels
*/
void getMatlErrors(std::vector<cv::Point>& centerline, double width, cv::Size rasterSize, const std::vector<cv::Point>& lEdgePts, const std::vector<cv::Point>& rEdgePts, std::vector<double>& errCL, std::vector<double>& errWD) {
	// Calculate Errors
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
		errCL.push_back((rEdge.at<float>(lnit.pos()) - lEdge.at<float>(lnit.pos())) / 2);
		errWD.push_back(lEdge.at<float>(lnit.pos()) + rEdge.at<float>(lnit.pos()) - MM2PIX(width));
	}
}