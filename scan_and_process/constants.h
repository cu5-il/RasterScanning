#pragma once

#define SAMPLING_TIME 0.25
#define NUM_DATA_SAMPLES 1800
#define NUM_DATA_SIGNALS 6
#define NUM_PROFILE_PTS 1600 // = 800 * Profile_out_update / SAMPLING_TIME  (Profile_out_update from LJ navigator "Common > Analog output > prof out update")
#define OUT_VOLTAGE -6
#define TASK_TRIG TASKID_04
#define OPAMP_GAIN -0.49875

#define SCAN_WIDTH 26.4
#define PIX2MM 0.02
#define RASTER_IMG_SIZE 700
#define PI 3.14159265

#define SCAN_OFFSET 5