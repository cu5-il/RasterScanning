#pragma once
#include <vector>
#include <opencv2/core.hpp>

#ifndef ERRORS_H
#define ERRORS_H

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
 * @param[in] targetWidths Vecor of desired width of the material in mm
 * @param[in] rasterSize Size of the raster image
 * @param[in] lEdgePts Points making up the left edge of the rod
 * @param[in] rEdgePts Points making up the right edge of the rod
 * @param[out] errCL Material centerline error in pixels
 * @param[out] errWD Material width error in pixels
*/
void getErrorsAt(std::vector<cv::Point>& waypoints, std::vector<double>targetWidths, cv::Size rasterSize, const std::vector<cv::Point>& lEdgePts, const std::vector<cv::Point>& rEdgePts, std::vector<double>& errCL, std::vector<double>& errWD);

#endif // !ERRORS_H