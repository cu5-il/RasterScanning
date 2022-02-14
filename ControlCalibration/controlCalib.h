#pragma once

#include <vector>
#include <string>
#include <opencv2/core.hpp>

#include "myTypes.h"
#include "myGlobals.h"
#include "raster.h"

#ifndef CONTROLCALIB_H
#define CONTROLCALIB_H

/**
 * @brief Modifies a parameter over a range of values for a raster path
 * @param path Vector of segmented path waypoints
 * @param test Parameter to modify. f (feedrate), a (auger voltage)
 * @param range starting and ending values of the parameter 
*/
void makeTestPath(std::vector<std::vector<Path>>& path, char test, double range[2]);

void analyzePrint(Raster raster, std::string filename);

void analyzePrint(Raster raster);

bool readTestParams(std::string filename, Raster& raster, double& wayptSpc, cv::Point3d& initPos, double& initVel, double& initExt, char& test, double range[2]);

#endif // !CONTROLCALIB_H
