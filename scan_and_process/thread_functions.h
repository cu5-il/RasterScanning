#pragma once

#include "raster.h"
#include "myTypes.h"

#ifndef THREAD_FNS_H
#define THREAD_FNS_H

void t_CollectScans(Raster raster);

void t_GetMatlErrors(Raster raster, double targetWidth);

void t_controller(std::vector<std::vector<Path>> pathSegs, int segsBeforeCtrl);

void t_printQueue(Path firstWpt);

void t_PollPositionFeedback(int rate);

#endif // THREAD_FNS_H