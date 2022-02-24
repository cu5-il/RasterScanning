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

void makePath(Raster raster, double wayptSpc, double theta, cv::Point3d initPos, double initVel, double initExt, std::vector<Segment>& seg, std::vector<std::vector<Path>>& path);

void readPath(std::string filename, double& rodLen, double& rodSpc, double& wayptSpc, std::deque<std::vector<double>>& path, std::deque<double>& theta);

/**
 * @brief Make a functionally graded scaffold pattern
 * @param path path to be modified
 * @param param Parameter to change. Either "f" to change the feedrate or "a" to change the auger torque
 * @param type Type of functionally graded scaffold. 1 = Bowtie, 2 = continuous gradient. (See Armstrong Dissertation pg 90)
 * @param range
*/
void makeFGS(std::vector<std::vector<Path>>& path, char param, int type, double range[2]);

#endif // !PATH_H