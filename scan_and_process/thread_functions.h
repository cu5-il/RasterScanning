#pragma once
#include<opencv2/core.hpp>
#include "raster.h"

#ifndef THREAD_FNS_H
#define THREAD_FNS_H

void t_CollectScans(Raster raster);

void t_GetMatlErrors(Raster raster);

void t_PollPositionFeedback(int rate);

#endif // THREAD_FNS_H