/* Function declarations*/
#pragma once


#include <vector> 

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>

/// @brief Processes the raw data signals.
/// 
/// 
/// @param[in]	data	array of signals where the rows are: [0] analog scanner output, [1] triggering signal, [2] x, [3] y, [4] z, and [5] theta position
/// @param[out]	fbk	structure with x, y, z, and theta coordinates of gantry when scan was taken
/// @param[out] profile	Z profile from scanner
///
void processData(double data[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& profile);


/// @brief Processes the raw data signals.
/// 
/// 
/// @param[in]	printROI	reference to vector
/// @param[out]	
/// @param[out] 
/// @param[out] 
///
void local2globalScan(int profileCols, const Coords fbk, const std::vector<double>& printROI, cv::Point& profileStart, cv::Point& profileEnd, cv::Range& profileROIRange);