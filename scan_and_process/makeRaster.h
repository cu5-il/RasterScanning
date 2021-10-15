#pragma once
#include <vector>
#include <opencv2/core.hpp>

void makeRaster(double length, double spacing, double border, double rodWidth, cv::Mat& raster, cv::Mat& edgeBoundary, std::vector<cv::Point>& coords);