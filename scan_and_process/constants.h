#pragma once
#include <cmath>

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define SAMPLING_TIME 0.25
#define NUM_DATA_SAMPLES 1800
#define NUM_DATA_SIGNALS 6
#define NUM_PROFILE_PTS 1600 // = 800 * Profile_out_update / SAMPLING_TIME  (Profile_out_update from LJ navigator "Common > Analog output > prof out update")
#define OUT_VOLTAGE -6
#define OPAMP_GAIN -0.49875
#define TASK_SCAN TASKID_04
#define TASK_PRINT TASKID_01
#define TASK_EXTRUDE TASKID_02

#define SCAN_WIDTH 26.4
#define RESOLUTION 0.05/*0.02*/  // 1 pixel = RESOLUTION mm
#define PIX2MM(pix)  (pix)*RESOLUTION
#define MM2PIX(mm)  std::lround( (mm) / RESOLUTION)
#define RASTER_IMG_WIDTH 11.0 //TODO: incorporate this into the makeRaster Function
#define PI 3.14159265

#define SCAN_OFFSET_X -13.0
#define SCAN_OFFSET_Y 0.7
#define VELOCITY 1

#define CVPLOT_HEADER_ONLY

#define AXES_ALL (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02 | AXISMASK_03)
#define DISPOSAL_X 00
#define DISPOSAL_Y 153
//#define LEADIN_LINE 5
//#define LEADOUT_LINE 5
#define SAFE_Z 5

//#define DEBUG_SCANNING

#endif // CONSTANTS_H