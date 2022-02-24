#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <string>
#include <algorithm>
#include <iterator> 
#include <valarray>
#include <deque>
#include <Windows.h>

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
#include "draw.h"
#include "gaussianSmooth.h"
#include "errors.h"
#include "A3200_functions.h"
#include "thread_functions.h"
#include "csvMat.h"
#include "raster.h"
#include <ctime>
#include "path.h"
#include "print.h"
#include "controlCalib.h"

std::string datetime(std::string format = "%Y.%m.%d-%H.%M.%S");

int main() {
	// Disable openCV warning in console
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
	// Set the output path with and prepend all file names with the time
	outDir.append(datetime("%Y.%m.%d") + "/");
	if (CreateDirectoryA(outDir.c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError()) {}
	else{
		std::cout << "Error creating output directory" << std::endl;
		system("pause");
		return 0;
	}

	std::thread t_scan, t_process, t_control, t_print;

	// Initialize parameters
	double initVel;
	double initExt;
	cv::Point3d initPos;
	double wayptSpc = 1;
	Raster raster;
	char testTp;
	double range[2];
	std::vector<std::vector<Path>> path;

	// Getting user input
	std::string resp, file;
	int lineNum;
	std::cout << "Select option: (p)rint, (s)can, or (a)nalyze data? " ;
	std::cin >> resp;
	if (resp.compare("a") == 0) {
		std::cout << "Enter test name: ";
		//std::cin >> file;
		file = "plate3";
		lineNum = 1;
		// read in test parameters and generate raster
		if (!readTestParams(std::string("./Input/" + file + ".txt"), raster, wayptSpc, initPos, initVel, initExt, testTp, range, lineNum)) { return 0; }
		makePath(raster, wayptSpc, 0, initPos, initVel, 0, segments, path);

		file = "./Output/2022.02.18/auger2/a10_17.44";
		cv::Mat edges = cv::Mat::zeros(raster.size(), CV_8U);
		readCSV(file + "_edges.csv", edges);
		edges.convertTo(edges, CV_8U);
		cv::imwrite(file + "_edgedata.png", edges);
		//edges = cv::imread(file + "_edgedata.png");
		//edges.convertTo(edges, CV_8U);

		// Analyzing the print
		outDir = file + "_";
		analyzePrint(raster, std::string(file + "_edgedata.png"));
		return 0;
	}
	else if (resp.compare("p") != 0 && resp.compare("s") != 0 ) {
		return 0;
	}
	std::cout << "Test #: ";
	std::cin >> lineNum;
	file = "plate2";
	// read in test parameters and generate raster
	if (!readTestParams(std::string("./Input/" + file + ".txt"), raster, wayptSpc, initPos, initVel, initExt, testTp, range, lineNum)) { return 0; }
	outDir.append(testTp + std::to_string(lineNum) + "_"+ datetime("%H.%M") + "_");


	// Creating the path and segmets
	if (resp.compare("p") == 0) {
		makePath(raster, wayptSpc, 0, initPos, initVel, initExt, segments, path);
		// Modifying the inputs
		makeTestPath(path, testTp, range);
	}
	else if (resp.compare("s") == 0) {
		initVel = 2;
		//Raster rasterScan = Raster(raster.length() - SCAN_OFFSET_X + raster.rodWidth(), raster.width(), raster.spacing(), raster.rodWidth());
		Raster rasterScan = Raster(raster.length() + 2 * raster.rodWidth(), raster.width(), raster.spacing(), raster.rodWidth());
		rasterScan.offset(cv::Point2d(initPos.x, initPos.y));
		rasterScan.offset(cv::Point2d(-SCAN_OFFSET_X - raster.rodWidth()));
		//initPos += cv::Point3d(0, 0, 2);
		makePath(rasterScan, wayptSpc, 0, initPos, initVel, 0, segments, path);
	}
	int segsBeforeCtrl = path.size();
	
	cv::Mat imSeg;
	drawSegments(raster.draw(), imSeg, segments, raster.origin(), 3);

	//goto cleanup;

	// Sanity check
	//system("pause");

	// A3200 Setup
	//=======================================
	//Connecting to the A3200
	{
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
		if (!A3200MotionHomeConditional(handle, TASKID_Library, (AXISMASK)(AXISMASK_00 | AXISMASK_01))) { A3200Error(); } // X & Y axes 
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
	}
	//=======================================
	
	//// notify scanner to start
	//q_scanMsg.push(true);
	//t_CollectScans(raster);

	// just printing, no scanning
	if (resp.compare("p") == 0){
		t_control = std::thread{ t_controller, path, segsBeforeCtrl };
		t_print = std::thread{ t_printQueue, path[0][0], true };
		t_print.join();
		t_control.join();
		goto cleanup;
	}
	// Scanning, no error calculations
	else if (resp.compare("s") == 0) {
		t_scan = std::thread{ t_CollectScans, raster };
		t_control = std::thread{ t_controller, path, segsBeforeCtrl };
		t_print = std::thread{ t_printQueue, path[0][0], false};
		t_print.join();
		t_scan.join();
		t_control.join();
		// calculate the average width of the segments
		segments.clear();
		makePath(raster, wayptSpc, 0, initPos, initVel, initExt, segments, path);
		analyzePrint(raster);
		goto cleanup;
	}

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

std::string datetime(std::string format) {
	time_t rawtime = time(NULL);
	struct tm timeinfo;
	char buffer[80];

	localtime_s(&timeinfo, &rawtime);
	strftime(buffer, 80, format.c_str(), &timeinfo);
	return std::string(buffer);
}