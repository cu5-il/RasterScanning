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

void interpPathPoints(std::vector<cv::Point2d> inPts, double wayptSpc, std::vector<cv::Point2d>& outPts);

void interpPathPoints(std::vector<cv::Point2i> inPts, double wayptSpc, std::vector<cv::Point2i>& outPts);

bool makePath(Raster raster, double wayptSpc, std::deque<double>& theta, cv::Point3d initPos, double initVel, double initExt, std::vector<Segment>& seg, std::vector<std::vector<Path>>& path);

void readPath(std::string filename, double& rodLen, double& rodSpc, double& wayptSpc, std::deque<std::vector<double>>& path, std::deque<double>& theta);

#endif // PATH_H