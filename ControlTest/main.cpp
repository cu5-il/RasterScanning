#include <iostream>
#include <fstream>
#include <iomanip>      // std::setw
#include <vector> 
#include <string>

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

#include "A3200_functions.h"
#include "thread_functions.h"
#include "raster.h"
#include <ctime>
#include "path.h"
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
	double initVel, scanVel;
	double initExt;
	cv::Point3d initPos;
	double wayptSpc = 1;
	Raster raster;
	std::string testTp;
	double range[2];
	std::vector<std::vector<Path>> path;
	int segsBeforeCtrl = 0;

	// defining the material models
	MaterialModel augerModel = MaterialModel(std::vector<double>{2, 3},
		std::vector<double>{4.15, 3.95},
		std::vector<double>{0.1, 0.1},
		std::vector<double>{-2.815, -2.815});

	// setting the print options
	double leadin = 5;
	double leadout = 5;
	PrintOptions printOpts(leadin);


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