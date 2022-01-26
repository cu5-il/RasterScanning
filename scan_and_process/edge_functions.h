#pragma once
#include <vector>
#include <opencv2/core.hpp>

#ifndef EDGE_FNS_H
#define EDGE_FNS_H

/**
 * @brief Breaks up the raster pattern into segments. Each vertical rod in the raster is a segment; corners are neglected.
 * @param[in] rasterCoords Vector of (x,y) points of the raster pattern
 * @param[in] ROIwidth of the ROI
 * @param[out] ROIs Vector of regions that are around each rod. Used to determine which edges belong to which segment
 * @param[out] centerlines Vector of point pairs defining the centerline of the region
 * @param[out] scanDonePts Point in the raster pattern when the region has been finished scanning. Set to midpoint of the following rod except for the last region, which is just the offset of the last point of the raster
*/
void makeSegments(const std::vector<cv::Point>& rasterCoords, double ROIwidth, std::vector<Segment>& seg, cv::Point2d initPos);

/**
 * @brief Finds the left and right edges of the material and smooths them
 * @param[in] segmentROI Rectangle specifying the region of the image to search for the edges
 * @param[in] gblEdges Image of edge points found by the scanner
 * @param[out] lEdgePts Filtered points that make up the edge in the left half of the ROI
 * @param[out] rEdgePts Filtered points that make up the edge in the right half of the ROI
 * @param[in] interp Flag indicating whether to interpolate the output edge points
*/
void getMatlEdges(const cv::Rect& segmentROI, const cv::Mat& gblEdges, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts, bool interp = false);

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
void getMatlErrors(std::vector<cv::Point>& centerline, double width, cv::Size rasterSize, const std::vector<cv::Point>& lEdgePts, const std::vector<cv::Point>& rEdgePts, std::vector<double>& errCL, std::vector<double>& errWD);

/**
 * @brief Calculates the material centerline and width errors at the input waypoints.
 * @param[in] waypoints Vector of waypoints in pixel coordinates where the errors should be calculated
 * @param[in] width Desired width of the material in mm
 * @param[in] rasterSize Size of the raster image
 * @param[in] lEdgePts Points making up the left edge of the rod
 * @param[in] rEdgePts Points making up the right edge of the rod
 * @param[out] errCL Material centerline error in pixels
 * @param[out] errWD Material width error in pixels
*/
void getErrorsAt(std::vector<cv::Point>& waypoints, double width, cv::Size rasterSize, const std::vector<cv::Point>& lEdgePts, const std::vector<cv::Point>& rEdgePts, std::vector<double>& errCL, std::vector<double>& errWD);

#endif // EDGE_FNS_H