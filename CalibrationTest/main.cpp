#include <iostream>
#include <fstream>
#include <iomanip>      // std::setw
//#include <cmath>
#include <vector> 
#include <string>
//#include <algorithm>
//#include <iterator> 
//#include <valarray>
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
//#include "gaussianSmooth.h"
//#include "errors.h"
#include "A3200_functions.h"
#include "thread_functions.h"
#include "csvMat.h"
#include "raster.h"
#include <ctime>
#include "path.h"
//#include "print.h"
#include "controlCalib.h"

std::string datetime(std::string format = "%Y.%m.%d-%H.%M.%S");

int main() {
	// Disable openCV warning in console
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
	// Set the output path with and prepend all file names with the time
	outDir.append(datetime("%Y.%m.%d") + "/");
	if (CreateDirectoryA(outDir.c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError()) {}
	else {
		std::cout << "Error creating output directory" << std::endl;
		system("pause");
		return 0;
	}

	//=======================================
	// Connecting to and setting up the A3200
	std::cout << "Connecting to A3200. Initializing if necessary." << std::endl;
	if (!A3200Connect(&handle)) { A3200Error(); }
	// Creating a data collection handle and setting up the data collection
	if (!A3200DataCollectionConfigCreate(handle, &DCCHandle)) { A3200Error(); }
	if (!setupDataCollection(handle, DCCHandle)) { A3200Error(); }
	// Initializing the extruder
	extruder = Extruder(handle, TASK_PRINT);
	//=======================================

	std::thread t_scan, t_process, t_control, t_print;

	// Initialize parameters
	double initVel;
	double initExt;
	cv::Point3d initPos;
	double wayptSpc = 1;
	Raster raster;
	std::string testTp;
	double range[2];
	std::vector<std::vector<Path>> path;
	int segsBeforeCtrl = 0;

	cv::Mat imSeg;

	// Getting user input
	std::string resp, file;
	int lineNum[2];
	file = "plate1";

	std::cout << "Select option: (p)rint, (s)can, or (m)ultiple? ";
	std::cin >> resp;
	if ( resp.compare("m") == 0) {
		std::cout << "Select option: (p)rint or (s)can? ";
		std::cin >> resp;
		std::cout << "Test # range: ";
		std::cin >> lineNum[0] >> lineNum[1];
	}
	else if (resp.compare("p") == 0 || resp.compare("s") == 0) {
		std::cout << "Test #: ";
		std::cin >> lineNum[0];
		lineNum[1] = lineNum[0];
	}
	else { return 0; }
	
	while (lineNum[0] <= lineNum[1]) {
		std::cout << "Test #: " << lineNum[0] << std::endl;
		outDir = "Output/" + datetime("%Y.%m.%d") + "/";
		// read in test parameters and generate raster
		if (!readTestParams(std::string("./Input/" + file + ".txt"), raster, wayptSpc, initPos, initVel, initExt, testTp, range, lineNum[0])) { return 0; }
		outDir.append(testTp + std::to_string(lineNum[0]) + "_" + datetime("%H.%M") + "_");

		// Creating the path and segmets
		if (resp.compare("p") == 0 || resp.compare("y") == 0) {
			makePath(raster, wayptSpc, 0, initPos, initVel, initExt, segments, path);
			// Modifying the inputs
			makeFGS(path, testTp[0], testTp[1], range);
			segsBeforeCtrl = path.size();
			// Sanity check
			//system("pause");

			//start printing
			t_control = std::thread{ t_controller, path, segsBeforeCtrl };
			t_print = std::thread{ t_printQueue, path[0][0], true };
			t_print.join();
			t_control.join();

			std::cout << "Scan the print? (y/n) ";
			std::cin >> resp;
			if (resp.compare("y") != 0) 
			{
				resp = "p";
			}
		}

		if (resp.compare("s") == 0 || resp.compare("y") == 0)
		{
			segments.clear();
			path.clear();
			initVel = 1;
			// make a raster pattern used for scanning
			Raster rasterScan = Raster(raster.length() + 2 * raster.rodWidth(), raster.width(), raster.spacing(), raster.rodWidth());
			//initPos += cv::Point3d(0, 0, 1);
			initPos += cv::Point3d(0, 2.5, 0);
			rasterScan.offset(cv::Point2d(initPos.x, initPos.y));
			rasterScan.offset(cv::Point2d(-SCAN_OFFSET_X - raster.rodWidth()));
			makePath(rasterScan, wayptSpc, 0, initPos, initVel, 0, segments, path);
			segsBeforeCtrl = path.size();
			// Sanity check
			//system("pause");

#ifdef DEBUG_SCANNING
			makePath(raster, wayptSpc, 0, initPos, initVel, initExt, segments, path);
			q_scanMsg.push(true);
			t_CollectScans(raster);
			return 0;
#endif // DEBUG_SCANNING

			// start scanning
			t_scan = std::thread{ t_CollectScans, raster };
			t_control = std::thread{ t_controller, path, segsBeforeCtrl };
			t_print = std::thread{ t_printQueue, path[0][0], false };
			t_scan.join();

			// Make the actual rater path used in the print
			segments.clear();
			path.clear();
			makePath(raster, wayptSpc, 0, initPos, initVel, initExt, segments, path);
			makeFGS(path, testTp[0], testTp[1], range);
			t_GetMatlErrors(raster, path);
			while (!q_errsMsg.empty()) {
				q_errsMsg.try_pop();
			}

			// Opening a file to save the results
			std::ofstream outfile;
			outfile.open(std::string(outDir + "pathData.txt").c_str());
			outfile.precision(3);

			// loop through each long segment
			for (int i = 0; i < path.size(); i += 2) {
				// loop through all the waypoints
				for (int j = 0; j < path[i].size(); j++) {
					outfile << std::setw(7) << std::fixed << path[i][j].x << "\t";
					outfile << std::setw(7) << std::fixed << path[i][j].y << "\t";
					outfile << std::setw(6) << std::fixed << path[i][j].f << "\t";
					outfile << std::setw(6) << std::fixed << path[i][j].e << "\t";
					outfile << std::setw(6) << std::fixed << path[i][j].w << "\t";
					outfile << std::setw(9) << std::fixed << segments[i].errWD()[j] << "\t";
					outfile << std::setw(9) << std::fixed << segments[i].errCL()[j] << "\n";
				}
			}
			outfile.close();

			t_print.join();
			t_control.join();
		}
		lineNum[0]++;
		segments.clear();
		path.clear();
	}
	
	drawSegments(raster.draw(), imSeg, segments, raster.origin(), 3);

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