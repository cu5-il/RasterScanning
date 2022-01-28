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

void postPrint(double speed, double augerSpeed) {
	double nextPos[2];
	// retract the auger 
	extruder.set(-0.7);

	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }
	// raise the z axis
	nextPos[0] = 10;
	if (!A3200MotionLinearVelocity(handle, TASK_PRINT, (AXISMASK)(AXISMASK_02), nextPos, 10)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); }
	// Rotate the theta axis to its starting position
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_03), 90, 20)) { A3200Error(); }
	//if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); } 
	// move to disposal cup
	nextPos[0] = DISPOSAL_X;
	nextPos[1] = DISPOSAL_Y;
	if (!A3200MotionLinearVelocity(handle, TASK_PRINT, (AXISMASK)(AXISMASK_00 | AXISMASK_01), nextPos, 10)) { A3200Error(); }
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); } 
}

void printPath(std::deque<std::vector<double>>& path, cv::Point2d initPos, double speed, double augerSpeed) {
	double nextPos[3];
	double queueLineCount;
	//char command[128];
	LPCSTR command[128];

	// End any program already running
	if (!A3200ProgramStop(handle, TASK_PRINT)) { A3200Error(); }

	// Put task into Queue mode.
	if (!A3200ProgramInitializeQueue(handle, TASK_PRINT)) { A3200Error(); }

	// Set VEOLCITY mode ON and put task in absolute mode 
	command[0] = "VELOCITY ON\nG90\n";
	if (!A3200CommandExecute(handle, TASK_PRINT, command[0], NULL)) { A3200Error(); }

	if (!A3200MotionSetupAbsolute(handle, TASK_PRINT)) { A3200Error(); }

	// enable the axes
	if (!A3200MotionEnable(handle, TASK_PRINT, AXES_ALL)) { A3200Error(); }

	// Move to initial position with lead in line
	std::cout << "Moving to initial position" << std::endl;
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_00), initPos.x - 5, 10)) { A3200Error(); }
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_01), initPos.y, 10)) { A3200Error(); }
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_03), 0, 30)) { A3200Error(); }
	if (!A3200MotionMoveAbs(handle, TASK_PRINT, (AXISINDEX)(AXISINDEX_02), -8/*-7.6*/, 3)) { A3200Error(); } //FIXME: manually set starting Z height
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_MoveDone, -1, NULL)) { A3200Error(); }

	// Turn on the extruder
	extruder.set(augerSpeed);

	// Fill the command queue
	std::cout << "Queue:" << std::endl;
	while (!path.empty()){
		// get the next coordinates
		nextPos[0] = path.front()[0] + initPos.x;
		nextPos[1] = path.front()[1] + initPos.y;
		nextPos[2] = path.front()[2] ;
		std::cout << std::fixed << nextPos[0] << ", " << nextPos[1] << ", " << nextPos[2] << std::endl;
		// move to the next coordinates
		while (!A3200MotionLinearVelocity(handle, TASK_PRINT, (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_03), nextPos, speed )) { 
			// If the command failed to load into the queue
			if (A3200GetLastError().Code == ErrorCode_QueueBufferFull) {
				// Wait if the Queue is full.
				std::cout << "Queue is full" << std::endl;
				Sleep(10);
			}
			else {
				A3200Error();
				break;
			}
		}
		// set the auger speed
		extruder.set(augerSpeed); //TODO: change this to a command or change the task to the same as the motion
		// pop the coordinates from the deque
		path.pop_front();

	}
	std::cout << "Path loaded, waiting for queue to empty" << std::endl;
	// wait for the queue to empty
	if( !A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }
	while (0 != (int)queueLineCount) {
		// If there are still commands to execute sleep, and then check again on how many
		// commands are left.
		Sleep(10);
		if (!A3200StatusGetItem(handle, TASK_PRINT, STATUSITEM_QueueLineCount, 0, &queueLineCount)) { A3200Error(); }
	}
	std::cout << "Path complete" << std::endl;
	// Stop using queue mode
	if (!A3200ProgramStop(handle, TASK_PRINT)) { A3200Error(); }
	// Disable the extruder
	extruder.disable();
	// Run the post print routine
	postPrint(speed, augerSpeed);
	// disable the axes
	if (!A3200MotionDisable(handle, TASK_PRINT, AXES_ALL)) { A3200Error(); } //FIXME: axes are not disabling
	std::cout << "Printing complete" << std::endl;
}