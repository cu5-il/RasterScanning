#pragma once
#include <vector>
#include <opencv2/core.hpp>

#ifndef MAKE_RASTER_H
#define MAKE_RASTER_H

void makeRaster(double length, double spacing, double border, double bdryWidth, cv::Mat& raster, cv::Mat& edgeBoundary, std::vector<cv::Point>& coords);

#endif // MAKE_RASTER_H