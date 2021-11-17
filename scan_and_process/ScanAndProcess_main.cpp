#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <string>
#include <algorithm>
#include <iterator> 
#include <valarray>

#include <opencv2/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "A3200.h"

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "scanner_functions.h"
#include "processing_functions.h"
#include "display_functions.h"
#include "makeRaster.h"
#include "gaussianSmooth.h"
#include "edge_functions.h"

int main();

// This function will print whatever the latest error was
void PrintError();
void A3200Error(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle);


int main() {
	A3200Handle handle = NULL;
	A3200DataCollectConfigHandle DCCHandle = NULL;
	AXISMASK axisMask = (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02);
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];

	// INITIALIZATION & Data collection
	//========================================================================================================================
	//Connecting to the A3200
	std::cout << "Connecting to A3200. Initializing if necessary.\n";
	if (!A3200Connect(&handle)) { PrintError(); /*goto cleanup;*/ }
	// Creating a data collection handle
	if (!A3200DataCollectionConfigCreate(handle, &DCCHandle)) { PrintError(); /*goto cleanup;*/ }
	// Setting up the data collection
	if (!setupDataCollection(handle, DCCHandle)) { PrintError(); /*goto cleanup;*/ }

	// Homing and moving the axes to the start position
	std::cout << "Enabling and homing X, Y, and then Z"<<std::endl;
	if (!A3200MotionEnable(handle, TASKID_Library, axisMask)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_00 | AXISMASK_01 ))) { PrintError(); /*goto cleanup;*/ } //home X & Y axes if not already done
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_02))) { PrintError(); /*goto cleanup;*/ } //home Z axis if not already done
	if (!A3200MotionWaitForMotionDone(handle, axisMask, WAITOPTION_InPosition, -1, NULL)) { PrintError(); /*goto cleanup;*/ }
	if (!A3200MotionDisable(handle, TASKID_Library, axisMask)) { PrintError(); /*goto cleanup;*/ }



	//========================================================================================================================
cleanup:

	if (NULL != handle) {
		printf("Disconnecting from the A3200.\n");
		if (!A3200Disconnect(handle)) { PrintError(); }
	}
	// Freeing the resources used by the data collection configuration
	if (NULL != DCCHandle) {
		if (!A3200DataCollectionConfigFree(DCCHandle)) { PrintError(); }
	}

#ifdef _DEBUG
	system("pause");
#endif
	return 0;
}

//===================================================================
//							FUNCTIONS
//===================================================================

void PrintError() {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	printf("Error : %s\n", data);
}

void A3200Error(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle) {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	printf("Error : %s\n", data);
	system("pause");

	if (NULL != handle) {
		printf("Disconnecting from the A3200.\n");
		if (!A3200Disconnect(handle)) { PrintError(); }
	}
	// Freeing the resources used by the data collection configuration
	if (NULL != DCCHandle) {
		if (!A3200DataCollectionConfigFree(DCCHandle)) { PrintError(); }
	}
}
