#include "myGlobals.h" /* Global variable declarations made available here */

A3200Handle handle = NULL;
A3200DataCollectConfigHandle DCCHandle = NULL;
Extruder extruder;

std::vector<Segment> segments;
threadsafe_queue<edgeMsg> q_edgeMsg;

extern int segmentNumScan = 0; // segment being scanned
extern int segmentNumError = 0; // segment that errors are being calculated for