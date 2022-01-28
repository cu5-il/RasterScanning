#pragma once
#include<opencv2/core.hpp>
#include "raster.h"
#include "myTypes.h"

#ifndef THREAD_FNS_H
#define THREAD_FNS_H

void t_CollectScans(Raster raster);

void t_GetMatlErrors(Raster raster, double targetWidth);

void t_controller(std::vector<std::vector<Path>> pathSegs, int segsBeforeCtrl);

void t_printQueue(cv::Point3d initPos);

void t_PollPositionFeedback(int rate);

#endif // THREAD_FNS_H