/* Function declarations*/
#pragma once

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>

void mouse_callback(int  event, int  x, int  y, int  flag, void* param);

cv::Mat showOverlay(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd, bool showImage);

cv::Mat showScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin, bool showImage);

cv::Mat showRaster(cv::Mat raster, cv::Mat gblEdges, bool showImage);

cv::Mat showAll(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd, cv::Mat locEdges, cv::Mat locWin, cv::Mat gblEdges, bool showImage);

cv::Mat plotScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin, bool showImage);