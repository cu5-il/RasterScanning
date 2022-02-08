#include <iostream>
#include <string>

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "A3200.h"
#include "A3200_functions.h"
#include<opencv2/core.hpp>

void prePrint(cv::Point3d initPos) {
	std::string cmd;

	// Clearing any messages
	if (!A3200CommandExecute(handle, TASK_PRINT, (LPCSTR)"MSGCLEAR -1", NULL)) { A3200Error(); }

	// retract the auger 
	extruder.set(-0.7);

	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }
	// raise the z axis
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_02), 5, 10)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); }
	// Move X, Y, and TH axes to their starting positions
	cmd = "G0 X " + std::to_string(initPos.x) + " Y " + std::to_string(initPos.y) + " TH 0" + " XF 10 YF 10 THF 20" ;
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); } 

	// Move Z axis to the starting position
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_02), initPos.z + 1, 10)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); }
#ifdef LEADIN_LINE
	if (!A3200MotionMoveInc(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_00), -LEADIN_LINE, 5)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); }
#endif // LEADIN_LINE
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_02), initPos.z, 1)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); }
}


void postPrint() {
	std::string cmd;

	// retract the auger 
	extruder.set(-0.7);

	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }
	// raise the z axis
	if (!A3200CommandExecute(handle, TASK_PRINT, (LPCSTR)"G1 Z5 F5 ", NULL)) { A3200Error(); }
	// Move X, Y, and TH axes to their final positions
	cmd = "G0 X " + std::to_string(DISPOSAL_X) + " Y " + std::to_string(DISPOSAL_Y) + " TH 90" + " XF 15 YF 15 THF 20";
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	// Disable extrusion
	extruder.disable();
}