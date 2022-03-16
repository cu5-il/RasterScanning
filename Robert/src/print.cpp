#include "print.h"
#include <iostream>
#include <string>
#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "A3200.h"
#include "A3200_functions.h"


void prePrint(Path firstWpt, PrintOptions printOpts) {
	std::string cmd;

	// End any programs already running
	//if (!A3200ProgramStop(handle, TASKID_Library)) { A3200Error(); }
	if (!A3200ProgramStop(handle, TASK_PRINT)) { A3200Error(); }

	// Disabling the auger and air
	if (!A3200IODigitalOutput(handle, TASK_PRINT, 0, AXISINDEX_00, 0)) { A3200Error(); } //equivalent to $WO[0].X = 0

	// Homing the axes if not already done
	if (!A3200MotionHomeConditional(handle, TASK_PRINT, (AXISMASK)(AXISMASK_03))) { A3200Error(); } // TH axis 
	if (!A3200MotionHomeConditional(handle, TASK_PRINT, (AXISMASK)(AXISMASK_02))) { A3200Error(); } // Z axis 
	if (!A3200MotionHomeConditional(handle, TASK_PRINT, (AXISMASK)(AXISMASK_00 | AXISMASK_01))) { A3200Error(); } // X & Y axes 
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); }

	// Clear the messages and the indicators in the CNC interface
	if (!A3200CommandExecute(handle, TASK_PRINT, (LPCSTR)"MSGCLEAR -1\n", NULL)) { A3200Error(); }
	for (int i = 1; i <= 6; i++) {
		if (!A3200CommandExecute(handle, TASK_PRINT, std::string("MSGLAMP " + std::to_string(i) + ", GRAY, \"\"\n").c_str(), NULL)) { A3200Error(); }
	}

	// Clearing any messages
	if (!A3200CommandExecute(handle, TASK_PRINT, (LPCSTR)"MSGCLEAR -1", NULL)) { A3200Error(); }

	// retract the auger 
	extruder.set(-0.7);

	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }
	// raise the z axis
	cmd = "G1 Z " + std::to_string(SAFE_Z) + " F5";
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	// Move X, Y, and TH axes to their starting positions
	cmd = "G0 X " + std::to_string(firstWpt.x) + " Y " + std::to_string(firstWpt.y) + " TH " + std::to_string(firstWpt.T) + " XF 20 YF 20 THF 20" ;
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); } 

	// Move Z axis to just above the starting position
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_02), firstWpt.z + 2, 10)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); }
	// make the lead in line
	if (printOpts.leadin > 0)
	{
		switch (segments.front().dir())
		{
		case 0: // positive x direction
			firstWpt.x -= printOpts.leadin;
			break;
		case 1: // positive y direction
			firstWpt.y -= printOpts.leadin;
			break;
		case 2: // negative x direction
			firstWpt.x += printOpts.leadin;
			break;
		case 3: // negative y direction
			firstWpt.y += printOpts.leadin;
			break;
		}
		cmd = "G1 X " + std::to_string(firstWpt.x) + " Y " + std::to_string(firstWpt.y) + " F 5 ";
		if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	}

	// move to the starting Z height
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_02), firstWpt.z, 1)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); }
}


void postPrint(Path lastWpt, PrintOptions printOpts) {
	std::string cmd;

	// retract the auger 
	extruder.set(-0.7);

	// make the lead out line
	if (printOpts.leadout > 0)
	{
		switch (segments.back().dir())
		{
		case 0: // positive x direction
			lastWpt.x += printOpts.leadout;
			break;
		case 1: // positive y direction
			lastWpt.y += printOpts.leadout;
			break;
		case 2: // negative x direction
			lastWpt.x -= printOpts.leadout;
			break;
		case 3: // negative y direction
			lastWpt.y -= printOpts.leadout;
			break;
		}
		lastWpt.z += 1;
		cmd = "G1 X " + std::to_string(lastWpt.x) + " Y " + std::to_string(lastWpt.y) + " Z " + std::to_string(lastWpt.z) + " F " + std::to_string(lastWpt.f);
		if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	}

	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }
	// raise the z axis
	cmd = "G1 Z " + std::to_string(SAFE_Z) + " F10";
	if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	// reduce auger retraction
	extruder.set(-0.7);
	
	if (printOpts.disposal)
	{
		// Move X, Y, and TH axes to their final positions
		cmd = "G0 X " + std::to_string(DISPOSAL_X) + " Y " + std::to_string(DISPOSAL_Y) + " TH 90" + " XF 20 YF 20 THF 20";
		if (!A3200CommandExecute(handle, TASK_PRINT, cmd.c_str(), NULL)) { A3200Error(); }
	}
	// Disable extrusion
	extruder.disable();
}