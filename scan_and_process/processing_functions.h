/* Function declarations*/
#pragma once

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>

/// @brief Processes the raw data signals.
/// 
/// 
/// @param[in]	data	array of signals where the rows are: [0] analog scanner output, [1] triggering signal, [2] x, [3] y, [4] z, and [5] theta position
/// @param[out]	fbk	structure with x, y, z, and theta coordinates of gantry when scan was taken
/// @param[out] profile	Z profile from scanner
/// @param[out] 
///
void processData(double collectedData[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& profile);