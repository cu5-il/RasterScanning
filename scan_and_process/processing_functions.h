/* Function declarations*/
#pragma once
#include <vector> 

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>

void getScan(double data[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& scan);

bool scan2ROI(cv::Mat& scan, const Coords fbk, const cv::Rect2d printROI, cv::Size rasterSize, cv::Mat& scanROI, cv::Point& scanStart, cv::Point& scanEnd);

void findEdges(cv::Mat edgeBoundary, cv::Point scanStart, cv::Point scanEnd, cv::Mat& scanROI, cv::Mat& gblEdges, cv::Mat& locEdges, cv::Mat& locWin, double heightThresh);