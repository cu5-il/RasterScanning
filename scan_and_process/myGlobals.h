/* Global variable declarations */

#pragma once

#include "myTypes.h"
#include "A3200.h"
#include <opencv2/core.hpp>
#include <vector> 
#include "threadsafeQueue.h"


A3200Handle handle = NULL;
A3200DataCollectConfigHandle DCCHandle = NULL;

cv::Mat raster, edgeBoundary;
std::vector<cv::Point> rasterCoords;

Coords fbk;
std::vector<double> printROI;

threadsafe_queue <cv::Mat> q_scannedEdges;

