#pragma once
#include <vector>
#include <opencv2/core.hpp>

void gaussianSmoothX(const std::vector<cv::Point>& unfiltPts, std::vector<cv::Point>& filtPts, int kSize, double sig);