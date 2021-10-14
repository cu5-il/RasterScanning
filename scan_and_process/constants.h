#pragma once

#define SAMPLING_TIME 0.25
#define NUM_DATA_SAMPLES 1800
#define NUM_DATA_SIGNALS 6
#define NUM_PROFILE_PTS 1600 // = 800 * Profile_out_update / SAMPLING_TIME  (Profile_out_update from LJ navigator "Common > Analog output > prof out update")
#define OUT_VOLTAGE -6
#define TASK_TRIG TASKID_04
#define OPAMP_GAIN -0.49875

#define SCAN_WIDTH 26.4
#define RESOLUTION 0.02  // 1 pixel = RESOLUTION mm
#define PIX2MM(pix)  pix*RESOLUTION
#define MM2PIX(mm)  std::round( (mm) / RESOLUTION)
#define RASTER_IMG_WIDTH 11.0 //TODO: incorporate this into the makeRaster Function
#define PI 3.14159265

#define SCAN_OFFSET 0

#define CVPLOT_HEADER_ONLY 