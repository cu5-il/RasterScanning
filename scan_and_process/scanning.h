/* Function declarations*/
#pragma once
#include "constants.h"
#include "myTypes.h"
#include "A3200.h"
#include <opencv2/core.hpp>

#ifndef SCANNING_H
#define SCANNING_H

bool setupDataCollection(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle);

/**
 * @brief Gets the analog profile from the scanner. This starts the data collection, sends a trigger signal to the scanner, then returns the collected data.
 * @param[in]  handle	The handle to the A3200
 * @param[in]  DCCHandle	The handle to an A3200 Data Collection Configuration object. If NULL, previous sent configuration will be used.
 * @param[out] data	The retrieved sample point in format data[signal][sample].
 * @return TRUE on success, FALSE if an error occurred. Call A3200GetLastError() for more information.
*/
bool collectData(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle, DOUBLE* data);

/**
 * @brief Extracts the scanned profile and position feedback from the collected data
 * @param[in] data array of signals where the rows are: [0] analog scanner output, [1] triggering signal, [2] x, [3] y, [4] z, and [5] theta position
 * @param[out] fbk structure with x, y, z, and theta coordinates of gantry when scan was taken
 * @param[out] scan	Z profile from scanner
*/
bool getScan(double data[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& scan);

/**
 * @brief Extracts the part of the scan that is within the print area defined by the printROI
 * @param[in] scan Profile from scanner
 * @param[in] fbk Position of the scanner (X,Y,Z,T) when the scan was taken
 * @param[in] printROI Global coordinates (in mm) defining where the print is. Vector in the form {Xmin, Ymin, Xmax, Ymax}
 * @param[in] rasterSize Size of the raster image
 * @param[out] scanROI Profile from the scanner that is within the print ROI
 * @param[out] scanStart Pixel coordinates of the start of the scan
 * @param[out] scanEnd Pixel coordinates of the end of the scan
 * @return TRUE if part of the scan is in the ROI, FALSE if the scan is outside of the ROI
*/
bool scan2ROI(cv::Mat& scan, const Coords fbk, const cv::Rect2d printROI, cv::Size rasterSize, cv::Mat& scanROI, cv::Point& scanStart, cv::Point& scanEnd);

/**
 * @brief Find the material edges in a single scan
 * @param[in] edgeBoundary Mask indicating where to search for the edges. Typically it is a dialated raster path
 * @param[in] scanStart Pixel coordinates of the start of the scan
 * @param[in] scanEnd Pixel coordinates of the end of the scan
 * @param[in] scanROI Profile from the scanner that is within the print ROI
 * @param[out] gblEdges Mat output showing the location of all the found edges in the global coordinate system
 * @param[out] locEdges Mat output the same size as scanROI showing where the edges were found on the scan
 * @param[out] locWin Image output the same size as scanROI showing where the search windows are on the scan
 * @param[out] heightThresh UNUSED Points below this thresholds will not be considered edges
*/
void findEdges(cv::Mat edgeBoundary, cv::Point scanStart, cv::Point scanEnd, cv::Mat& scanROI, cv::Mat& edges, double heightThresh, int order = 1);

void findEdges2(cv::Mat edgeBoundary, cv::Point scanStart, cv::Point scanEnd, cv::Mat& scanROI, cv::Mat& edges);

#endif // !SCANNING_H