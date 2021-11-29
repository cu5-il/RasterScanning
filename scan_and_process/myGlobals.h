/* Global variable declarations */

#pragma once
#include <vector> 
#include <atomic>
#include "myTypes.h"
#include "A3200.h"
#include <opencv2/core.hpp>
#include "threadsafeQueue.h"


extern A3200Handle handle;
extern A3200DataCollectConfigHandle DCCHandle;

extern std::vector<Segment> segments;
extern threadsafe_queue<edgeMsg> q_edgeMsg;

extern int segmentNumScan;
extern int segmentNumError;