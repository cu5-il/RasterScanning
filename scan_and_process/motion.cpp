#include <iostream>
#include <vector>
#include <deque>
#include <iterator>

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "A3200.h"
#include "A3200_functions.h"
#include <opencv2/core.hpp>
#include "extrusion.h"

void printPath(std::deque<std::vector<double>>& path, cv::Point2d initPos, double speed, double augerSpeed) {
	Extruder extruder;
	double nextPos[3];

	//Set motion commands to be in absolute mode
	if (!A3200MotionSetupAbsolute(handle, TASK_MOVE)) { A3200Error(); }
	std::cout << "Begin printing" << std::endl;
	// enable the axes
	if (!A3200MotionEnable(handle, TASK_MOVE, AXES_ALL)) { A3200Error(); }
	// loop through all points in the path
	while (!path.empty()){
		// get the next coordinates
		nextPos[0] = path.front()[0] + initPos.x;
		nextPos[1] = path.front()[1] + initPos.y;
		nextPos[2] = path.front()[2];
		// move to the next coordinates
		if (!A3200MotionLinearVelocity(handle, TASK_MOVE, (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02), nextPos, speed )) { A3200Error(); }
		// set the auger speed
		extruder.set(augerSpeed);
		// pop the coordinates from the deque
		path.pop_front();
		// wait for the motion to be done before moving to the next coordinates
		if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); } // not sure if WAITOPTION_InPosition should be used or WAITOPTION_MoveDone
	}
	std::cout << "Path complete" << std::endl;
	// retract the auger 
	extruder.set(-augerSpeed);
	// raise the z axis
	if (!A3200MotionMoveInc(handle, TASK_MOVE, (AXISINDEX)(AXISINDEX_02), 20, 10)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); } // not sure if WAITOPTION_InPosition should be used or WAITOPTION_MoveDone
	// move to disposal cup
	if (!A3200MotionSetupAbsolute(handle, TASK_MOVE)) { A3200Error(); }
	nextPos[0] = DISPOSAL_X;
	nextPos[1] = DISPOSAL_Y;
	if (!A3200MotionLinearVelocity(handle, TASK_MOVE, (AXISMASK)(AXISMASK_00 | AXISMASK_01), nextPos, speed)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); } // not sure if WAITOPTION_InPosition should be used or WAITOPTION_MoveDone
	// Disable the extruder
	extruder.disable();
	// disable the axes
	if (!A3200MotionDisable(handle, TASK_MOVE, AXES_ALL)) { A3200Error(); }
	std::cout << "Printing complete" << std::endl;
}

