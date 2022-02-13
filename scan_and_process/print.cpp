#include <iostream>
#include <string>

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "A3200.h"
#include "A3200_functions.h"


void prePrint(Path firstWpt) {
	std::string cmd;

	// Clearing any messages
	if (!A3200CommandExecute(handle, TASK_PRINT, (LPCSTR)"MSGCLEAR -1", NULL)) { A3200Error(); }

	// retract the auger 
	extruder.set(-0.7);

	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }
	// raise the z axis
	cmd = "G1 Z " + std::to_string(SAFE_Z) + " F5";
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	// Move X, Y, and TH axes to their starting positions
	cmd = "G0 X " + std::to_string(firstWpt.x) + " Y " + std::to_string(firstWpt.y) + " TH " + std::to_string(firstWpt.T) + " XF 10 YF 10 THF 20" ;
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); } 

	// Move Z axis to the starting position
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_02), firstWpt.z + 2, 10)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); }
#ifdef LEADIN_LINE
	if (!A3200MotionMoveInc(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_00), -LEADIN_LINE, 5)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); }
#endif // LEADIN_LINE
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_02), firstWpt.z, 1)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); }
}


void postPrint() {
	std::string cmd;

	// retract the auger 
	extruder.set(-0.7);

	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }
	// raise the z axis
	cmd = "G1 Z " + std::to_string(SAFE_Z) + " F5";
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	// Move X, Y, and TH axes to their final positions
	cmd = "G0 X " + std::to_string(DISPOSAL_X) + " Y " + std::to_string(DISPOSAL_Y) + " TH 90" + " XF 15 YF 15 THF 20";
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	// Disable extrusion
	extruder.disable();
}