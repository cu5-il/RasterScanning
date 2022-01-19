#pragma once
#include <vector>
#include <opencv2/core.hpp>

#ifndef GAUSSIAN_SMOOTH_H
#define GAUSSIAN_SMOOTH_H

void gaussianSmoothX(const std::vector<cv::Point>& unfiltPts, std::vector<cv::Point>& filtPts, int kSize, double sig);

void gaussianSmoothY(const std::vector<cv::Point>& unfiltPts, std::vector<cv::Point>& filtPts, int kSize, double sig);

#endif // GAUSSIAN_SMOOTH_H