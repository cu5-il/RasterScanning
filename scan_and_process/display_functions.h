/* Function declarations*/
#pragma once

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>

void mouse_callback(int  event, int  x, int  y, int  flag, void* param);

void showOverlay(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd);

void showScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin);

void showRaster(cv::Mat& raster, cv::Mat gblEdges);