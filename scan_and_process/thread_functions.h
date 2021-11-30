#pragma once
#include<opencv2/core.hpp>

#ifndef THREAD_FNS_H
#define THREAD_FNS_H

void t_CollectScans(const cv::Mat raster, const cv::Mat edgeBoundary, cv::Rect2d printROI);

void t_GetMatlErrors(const cv::Mat raster);

void t_PollPositionFeedback(int rate);

#endif // THREAD_FNS_H