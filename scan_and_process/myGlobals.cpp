#include "myGlobals.h" /* Global variable declarations made available here */

A3200Handle handle = NULL;
A3200DataCollectConfigHandle DCCHandle = NULL;

std::vector<Segment> segments;
threadsafe_queue<edgeMsg> q_edgeMsg;
bool doneScanning = false;
bool positionFlag = false;


extern int segmentScan = 0; // segment being scanned
extern int segmentError = 0; // segment that errors are being calculated for