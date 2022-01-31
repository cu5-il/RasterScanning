#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <string>
#include <algorithm>
#include <iterator> 
#include <valarray>
#include <deque>

#include <opencv2/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/core/utils/logger.hpp"

#include "A3200.h"

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "scanning.h"
#include "display_functions.h"
#include "gaussianSmooth.h"
#include "errors.h"
#include "A3200_functions.h"
#include "thread_functions.h"
#include "csvMat.h"
#include "motion.h"
#include "raster.h"
#include <ctime>
#include "path.h"

std::string datetime();

int main() {
	// Disable openCV warning in console
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
	// Set the output path with and prepend all file names with the time
	outDir.append(datetime() + "_");

	std::thread t_scan, t_process, t_control, t_print;

	// Defining the initial parameters
	
	double initVel = 3;
	double initExt = 0.7;
	//cv::Point3d initPos = cv::Point3d(0, 0, 0);
	cv::Point3d initPos = cv::Point3d(45, 15, -8);
	//initPos = cv::Point3d(95, 15, 0);
	double targetWidth = .5;

	//Load the raster path generated in Matlab
	double rodLen, rodSpc, rodWidth, wayptSpc;
	std::deque<std::vector<double>> xyTpath;
	std::deque<double> theta;
	readPath("Input/pathCoords.txt", rodLen, rodSpc, wayptSpc, xyTpath, theta);
	xyTpath.clear();
	
	// Make raster
	rodWidth = 3;
	Raster raster = Raster(rodLen, rodSpc, rodWidth);
	raster.offset(cv::Point2d(initPos.x, initPos.y));

	// Creating the path and segmets
	int segsBeforeCtrl = 3;
	std::vector<std::vector<Path>> path;
	if (!makePath(raster, wayptSpc, theta, initPos, initVel, initExt, segments, path)) { return -1; }
	//segsBeforeCtrl = path.size();

//goto LoadData;

	// A3200 Setup
	//=======================================
	//Connecting to the A3200
	std::cout << "Connecting to A3200. Initializing if necessary." << std::endl;
	if (!A3200Connect(&handle)) { A3200Error(); }
	// Creating a data collection handle and setting up the data collection
	if (!A3200DataCollectionConfigCreate(handle, &DCCHandle)) { A3200Error(); }
	if (!setupDataCollection(handle, DCCHandle)) { A3200Error(); }
	// Disabling the auger and air
	if (!A3200IODigitalOutput(handle, TASKID_Library, 0, AXISINDEX_00, 0)) { A3200Error(); } //equivalent to $WO[0].X = 0
	// Homing the axes if not already done
	std::cout << "Homing axes." << std::endl;
	if (!A3200MotionEnable(handle, TASKID_Library, AXES_ALL)) { A3200Error(); }
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_03))) { A3200Error(); } // TH axis 
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_02))) { A3200Error(); } // Z axis 
	if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_00 | AXISMASK_01 ))) { A3200Error(); } // X & Y axes 
	if (!A3200MotionWaitForMotionDone(handle, AXES_ALL, WAITOPTION_InPosition, -1, NULL)) { A3200Error(); }
	if (!A3200MotionDisable(handle, TASKID_Library, AXES_ALL)) { A3200Error(); }
	// End any program already running
	if (!A3200ProgramStop(handle, TASK_PRINT)) { A3200Error(); }
	// Initializing the extruder
	extruder = Extruder(handle, TASK_PRINT);
	// Clear the messages and the indicators in the CNC interface
	if (!A3200CommandExecute(handle, TASK_PRINT, (LPCSTR)"MSGCLEAR -1\n", NULL)) { A3200Error(); }
	for (int i = 1; i <= 6; i++) {
		if (!A3200CommandExecute(handle, TASK_PRINT, std::string("MSGLAMP " + std::to_string(i) + ", GRAY, \"\"\n").c_str(), NULL)) { A3200Error(); }
	}
	//=======================================

	//goto cleanup;
	//t_controller(path, segsBeforeCtrl);
	//t_queueCmds();
	//goto cleanup;

	t_scan = std::thread{ t_CollectScans, raster };
	t_process = std::thread{ t_GetMatlErrors, raster, targetWidth };
	t_control = std::thread{ t_controller, path, segsBeforeCtrl };
	t_print = std::thread{ t_printQueue, initPos };

	t_print.join();
	t_scan.join();
	t_process.join();
	t_control.join();

	//goto cleanup;

LoadData:
	// DEBUGGING ERROR CALCULATIONS
	//edgeMsg msg;
	//cv::Mat edges = cv::Mat::zeros(raster.size(), CV_8U);
	//readCSV("C:/Users/cu5/source/repos/RasterScanning/scan_and_process/Output/2022.01.20-15.14.37_edges.csv", edges);
	//edges.convertTo(edges, CV_8U);
	//for (int i = 0; i < segments.size(); i++) {
	//	msg.addEdges(edges, i, (i == segments.size() - 1));
	//	q_edgeMsg.push(msg);
	//}
	//t_GetMatlErrors(raster, targetWidth);
	
cleanup:
	//A3200 Cleanup
	//=======================================
	// Freeing the resources used by the data collection configuration
	if (NULL != DCCHandle) {
		if (!A3200DataCollectionConfigFree(DCCHandle)) { A3200Error(); }
	}
	// Disconnecting from the A3200
	if (NULL != handle) {
		printf("Disconnecting from the A3200.\n");
		if (!A3200Disconnect(handle)) { A3200Error(); }
	}

#ifdef _DEBUG

#endif
	return 0;
}

cv::Mat translateImg(cv::Mat& img, int offsetx, int offsety) {
	cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
	cv::warpAffine(img, img, trans_mat, img.size());
	return img;
}

std::string datetime(){
	time_t rawtime = time(NULL);
	struct tm timeinfo;
	char buffer[80];

	localtime_s(&timeinfo, &rawtime);

	strftime(buffer, 80, "%Y.%m.%d-%H.%M.%S", &timeinfo);
	return std::string(buffer);
}