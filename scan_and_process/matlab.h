#pragma once
#include <string>
#include <vector>
#include <deque>
#include "raster.h"


#ifndef MATLAB_H
#define MATLAB_H

void export2matlab(std::string filename, Raster& raster);

void readPath(std::string filename, double& rodLength, double& rodSpacing, std::deque<std::vector<double>>& path);

#endif // MATLAB_H