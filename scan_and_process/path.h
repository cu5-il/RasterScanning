#pragma once

#include <vector>
#include <deque>
#include <opencv2/core.hpp>

#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"
#include "raster.h"

#ifndef PATH_H
#define PATH_H

void readPath(std::string filename, double& rodLength, double& rodSpacing, std::deque<std::vector<double>>& path);

void segmentPath(Raster raster, double ROIwidth, std::vector<Segment>& seg, cv::Point2d initPos);

void interpolatePath(Raster raster, double waypointSpacing, std::vector<std::vector<cv::Point2d>>& path_mm, std::vector<std::vector<cv::Point>>& path_px);

#endif // PATH_H