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
#define TASK_MOVE TASKID_01
#define TASK_EXTRUDE TASKID_02

#define SCAN_WIDTH 26.4
#define RESOLUTION 0.02  // 1 pixel = RESOLUTION mm
#define PIX2MM(pix)  pix*RESOLUTION
#define MM2PIX(mm)  std::round( (mm) / RESOLUTION)
#define RASTER_IMG_WIDTH 11.0 //TODO: incorporate this into the makeRaster Function
#define PI 3.14159265

#define SCAN_OFFSET 13.5
#define VELOCITY 1

#define CVPLOT_HEADER_ONLY

#define AXES_ALL (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02 | AXISMASK_03)
#define DISPOSAL_X 30
#define DISPOSAL_Y 160

#endif // CONSTANTS_H