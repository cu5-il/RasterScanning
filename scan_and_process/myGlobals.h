/* Global variable declarations */

#pragma once
#include <vector> 
#include <atomic>
#include <string>
#include "myTypes.h"
#include "A3200.h"
#include <opencv2/core.hpp>
#include "threadsafeQueue.h"
#include "extrusion.h"

#ifndef MY_GLOBALS_H
#define MY_GLOBALS_H

extern A3200Handle handle;
extern A3200DataCollectConfigHandle DCCHandle;

extern Extruder extruder;

extern std::vector<Segment> segments;
extern threadsafe_queue<edgeMsg> q_edgeMsg;

extern int segmentNumScan;
extern int segmentNumError;

extern std::string outDir;

#endif // MY_GLOBALS_H