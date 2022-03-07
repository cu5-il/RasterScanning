#pragma once

#include <vector>
#include <string>
#include <opencv2/core.hpp>

#include "myTypes.h"
#include "myGlobals.h"
#include "raster.h"
#include "MaterialModel.h"

#ifndef CONTROLCALIB_H
#define CONTROLCALIB_H

/**
 * @brief Modifies a parameter over a range of values for a raster path
 * @param path Vector of segmented path waypoints
 * @param test Parameter to modify. f (feedrate), a (auger voltage)
 * @param range starting and ending values of the parameter 
*/
void makeCalibPath(std::vector<std::vector<Path>>& path, char test, double range[2]);

//void analyzePrint(Raster raster, std::string filename);

void analyzePrint(Raster raster);

bool readTestParams(std::string filename, Raster& raster, double& wayptSpc, cv::Point3d& initPos, double& initVel, double& initExt, char& test, double range[2]);

bool readTestParams(std::string filename, Raster& raster, double& wayptSpc, cv::Point3d& initPos, double& initVel, double& initExt, std::string& test, double range[2], int lineNum);

/**
 * @brief Make a functionally graded scaffold pattern
 * @param path path to be modified
 * @param param Parameter to change. Either "f" to change the feedrate or "a" to change the auger torque
 * @param type Type of functionally graded scaffold. 'b' = Bowtie, 'g' = continuous gradient. (See Armstrong Dissertation pg 90)
 * @param range
*/
void makeFGS(std::vector<std::vector<Path>>& path, char param, char type, double range[2], MaterialModel model);

#endif // !CONTROLCALIB_H
