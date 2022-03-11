#pragma once
#include <cmath>

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define TASK_SCAN TASKID_04
#define TASK_PRINT TASKID_01
#define AXES_ALL (AXISMASK)(AXISMASK_00 | AXISMASK_01 | AXISMASK_02 | AXISMASK_03)

#define RESOLUTION 0.05/*0.02*/  // 1 pixel = RESOLUTION mm
#define PIX2MM(pix)  (pix)*RESOLUTION
#define MM2PIX(mm)  std::lround( (mm) / RESOLUTION)
#define PI 3.14159265

#define SCAN_OFFSET_X -15.21
#define SCAN_OFFSET_Y 1.55

#define CVPLOT_HEADER_ONLY
//#define DEBUG_SCANNING

// Disable min and max macros to avoid build errors
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#endif // CONSTANTS_H