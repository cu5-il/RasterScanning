/* Global variable declarations */

#pragma once

#include "myTypes.h"
#include "A3200.h"
#include <opencv2/core.hpp>
#include <vector> 
#include "threadsafeQueue.h"


extern A3200Handle handle;
extern A3200DataCollectConfigHandle DCCHandle;

extern threadsafe_queue <cv::Mat> q_scannedEdges;
extern bool doneScanning = false;
extern bool positionFlag = false; 

extern int segmentNum = 0;