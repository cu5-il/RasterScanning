#pragma once
#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <vector>

#ifndef CVPLOT_FNS_H
#define CVPLOT_FNS_H

cv::Mat plotScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin, bool showImage = false);

cv::Mat plotScan(cv::Mat scanROI, std::vector<cv::Mat> mats);

void plotEdges(const std::vector<cv::Point>& unfiltered, const std::vector<cv::Point>& filtered);

#endif // CVPLOT_FNS_H