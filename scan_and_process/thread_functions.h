#pragma once
#include<opencv2/core.hpp>

void t_CollectScans(const cv::Mat raster, const cv::Mat edgeBoundary, cv::Rect2d printROI);

void t_GetMatlErrors(const cv::Mat raster);

void t_PollPositionFeedback(int rate);