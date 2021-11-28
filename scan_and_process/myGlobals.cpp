#include "myGlobals.h" /* Global variable declarations made available here */

A3200Handle handle = NULL;
A3200DataCollectConfigHandle DCCHandle = NULL;

threadsafe_queue <cv::Mat> q_scannedEdges;
bool doneScanning = false;
bool positionFlag = false;

int segmentNum = 0;