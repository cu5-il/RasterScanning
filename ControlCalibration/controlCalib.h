#pragma once

#include <vector>
#include <opencv2/core.hpp>

#include "myTypes.h"
#include "myGlobals.h"
#include "raster.h"

#ifndef CONTROLCALIB_H
#define CONTROLCALIB_H

/**
 * @brief Modifies a parameter over a range of values for a raster path
 * @param path Vector of segmented path waypoints
 * @param param Parameter to modify. 0 = f (feedrate), 1 = e (auger voltage)
 * @param range starting and ending values of the parameter 
*/
void makeTestPath(std::vector<std::vector<Path>>& path, int param, double range[2]);

#endif // !CONTROLCALIB_H
