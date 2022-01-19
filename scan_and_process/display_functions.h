/* Function declarations*/
#pragma once
#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>

#ifndef DISPLAY_FNS_H
#define DISPLAY_FNS_H

void mouse_callback(int  event, int  x, int  y, int  flag, void* param);

cv::Mat showOverlay(cv::Mat raster, cv::Mat scanROI, const Coords fbk, cv::Point scanStart, cv::Point scanEnd, bool showImage = false);

cv::Mat showScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin, bool showImage = false);

cv::Mat showRaster(cv::Mat raster, cv::Mat gblEdges, const cv::Scalar& color, const int pointSz = 1, bool showImage = false);

cv::Mat showAll(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd, cv::Mat locEdges, cv::Mat locWin, cv::Mat gblEdges, bool showImage = false);

void addScale(cv::Mat& image, cv::Point offset = cv::Point(25, 25));

void showErrors(cv::InputArray src, cv::OutputArray dst, std::vector<Segment>& seg);

#endif // DISPLAY_FNS_H