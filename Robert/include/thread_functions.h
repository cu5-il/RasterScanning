#pragma once

#include "raster.h"
#include "myTypes.h"
#include "controller.h"

#ifndef THREAD_FNS_H
#define THREAD_FNS_H

void t_CollectScans(Raster raster);

void t_GetMatlErrors(Raster raster, std::vector<std::vector<Path>> path);

void t_noController(std::vector<std::vector<Path>> path);

void t_controller(std::vector<std::vector<Path>>& path, Controller& controller);

void t_printQueue(Path firstWpt, PrintOptions printOpts);

void t_PollPositionFeedback(int rate);

#endif // THREAD_FNS_H