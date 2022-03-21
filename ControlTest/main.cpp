#include <iostream>
#include <fstream>
#include <iomanip>      // std::setw
#include <vector> 
#include <string>
#include <numeric> // std::accumulate

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
	double initVel;
	double initExt;
	cv::Point3d initPos;
	double wayptSpc = 1;
	Raster raster;
	std::string testTp;
	double range[2];
	std::vector<std::vector<Path>> path, ctrlPath;
	std::deque<double> theta;

	// defining the material models
	MaterialModel augerModel = MaterialModel('a',
		std::vector<double>{2, 3},
		std::vector<double>{4.15, 3.95},
		std::vector<double>{0.1, 0.1},
		std::vector<double>{-2.815, -2.815});

	// setting up the controller
	AugerController controller(augerModel, 0.2, 3.5);

	// setting the print options
	double leadin = 5;
	PrintOptions printOpts(leadin);
	printOpts.extrude = true;
	printOpts.disposal = false;
	printOpts.asyncTheta = 32;

	// Getting user input
	std::string resp, file;
	char option;
	int lineNum;
	file = "plate";

	std::cout << "Select option: (p)rint or (s)can? ";
	std::cin >> option;
	std::cout << "Test #: ";
	std::cin >> lineNum;

	outDir = "Output/" + datetime("%Y.%m.%d") + "/";
	// read in test parameters and generate raster
	if (!readTestParams(std::string("./Input/" + datetime("%Y.%m.%d") + "/" + file + ".txt"), raster, wayptSpc, initPos, initVel, initExt, testTp, range, lineNum)) { return 0; }
	outDir.append(testTp + "_" + std::to_string(lineNum) + "_" + datetime("%H.%M") + "_");

	// read in the theta path or use the asynchronous theta path
	if (printOpts.asyncTheta == 0) {
		// read in the theta coordinates
		file = "./Input/path/pathCoords_" + std::to_string((int)raster.length()) + "x" + std::to_string((int)raster.width()) + "x" + std::to_string((int)raster.spacing()) + "_v" + std::to_string((int)initVel) + ".txt";
		readTheta(file, theta);
		makePath(raster, wayptSpc, theta, initPos, initVel, initExt, segments, path);
	}
	else {
		makePath(raster, wayptSpc, 0, initPos, initVel, initExt, segments, path);
	}
	
	cv::Mat imSeg;
	drawSegments(raster.draw(), imSeg, segments, raster.origin(), 3);

	// make the path
	makeFGS(path, testTp[0], testTp[1], range, augerModel);

	// add a lead out line
	printOpts.leadout = -SCAN_OFFSET_X + 1;
	switch (segments.back().dir())
	{
	case 0: // positive x direction
		segments.back().setScanDonePt(segments.back().scanDonePt() - cv::Point2d(SCAN_OFFSET_X, 0));
		break;
	case 2: // negative x direction
		segments.back().setScanDonePt(segments.back().scanDonePt() + cv::Point2d(SCAN_OFFSET_X, 0));
		break;
	}

	switch (option)
	{
	default:
		return 0;
	case 'p': // PRINTING
		ctrlPath = path;

		t_scan = std::thread{ t_CollectScans, raster };
		t_process = std::thread{ t_GetMatlErrors, raster, path };
		t_print = std::thread{ t_printQueue, path[0][0], printOpts };
		t_control = std::thread{ t_controller, std::ref(ctrlPath), std::ref(controller) };

		t_scan.join();
		t_process.join();
		t_control.join();

		break;
	
	case 's': // SCANNING
		Raster rasterScan;
		segments.clear();
		path.clear();
		printOpts.extrude = false;
		initPos += cv::Point3d(0, 0, 1); // raise path
		std::cout << "(m)oving or (f)ixed scan: ";
		std::cin >> option;
		switch (option)
		{
		default:
			return 0;
		case 'm':
			rasterScan = raster;
			rasterScan.offset(cv::Point2d(0, 1.5)); // offset path in y direction
			makePath(rasterScan, wayptSpc, 0, initPos, initVel, initExt, segments, path);
			makeFGS(path, testTp[0], testTp[1], range, augerModel);
			ctrlPath = path;
			t_scan = std::thread{ t_CollectScans, raster };
			t_process = std::thread{ t_GetMatlErrors, raster, path };
			t_print = std::thread{ t_printQueue, path[0][0], printOpts };
			t_control = std::thread{ t_controller, std::ref(ctrlPath), std::ref(controller) };

			t_scan.join();
			t_process.join();
			t_control.join();
			break;
		case 'f':
			printOpts.leadin = 0;
			printOpts.leadout = 0;
			printOpts.asyncTheta = 0;
			rasterScan = Raster(raster.length() + 2 * raster.rodWidth(), raster.width(), raster.spacing(), raster.rodWidth());
			rasterScan.offset(cv::Point2d(initPos.x, initPos.y));
			rasterScan.offset(cv::Point2d(0, 1.5));

			std::cout << "Scan at (1) 0 deg or (2) 180 deg: ";
			std::cin >> option;
			switch (option)
			{
			default:
				return 0;
			case '1':
				rasterScan.offset(cv::Point2d(-SCAN_OFFSET_X - raster.rodWidth(), 0));
				makePath(rasterScan, wayptSpc, 0, initPos, initVel, initExt, segments, path);
				break;
			case '2':
				rasterScan.offset(cv::Point2d(SCAN_OFFSET_X - raster.rodWidth(), 0));
				makePath(rasterScan, wayptSpc, 180, initPos, initVel, initExt, segments, path);
				break;
			}

			ctrlPath = path;
			t_scan = std::thread{ t_CollectScans, raster };
			t_print = std::thread{ t_printQueue, path[0][0], printOpts };
			t_control = std::thread{ t_noController, ctrlPath };

			t_scan.join();
			t_control.join();

			// Remake the actual rater path used in the print
			segments.clear();
			path.clear();
			makePath(raster, wayptSpc, 0, initPos, initVel, initExt, segments, path);
			makeFGS(path, testTp[0], testTp[1], range, augerModel);
			t_GetMatlErrors(raster, path);
			break;
		}

		break;
	}

	// Opening a file to save the results
	std::ofstream outfile;
	outfile.open(std::string(outDir + "pathData.txt").c_str());
	outfile.precision(3);
	// loop through each long segment
	for (int i = 0; i < path.size(); i += 2) {
		// loop through all the waypoints
		for (int j = 0; j < path[i].size(); j++) {
			outfile << std::setw(7) << std::fixed << ctrlPath[i][j].x << "\t";
			outfile << std::setw(7) << std::fixed << ctrlPath[i][j].y << "\t";
			outfile << std::setw(6) << std::fixed << ctrlPath[i][j].f << "\t";
			outfile << std::setw(6) << std::fixed << ctrlPath[i][j].e << "\t";
			outfile << std::setw(6) << std::fixed << ctrlPath[i][j].w << "\t";
			if (!segments[i].errWD().empty())
				outfile << std::setw(9) << std::fixed << segments[i].errWD()[j] << "\t";
			if (!segments[i].errCL().empty())
				outfile << std::setw(9) << std::fixed << segments[i].errCL()[j];
			outfile << "\n";
		}
	}
	outfile.close();

	// calculate the error norms
	double E2d;
	auto sumsq = [](double a, double b) {
		if (!std::isnan(b)) return a + b * b;
		else return a; };
	outfile.open(std::string(outDir + "errors.txt").c_str());
	outfile.precision(3);
	// loop through each long segment
	for (int i = 0; i < path.size(); i += 2) {
		E2d = sqrt(std::accumulate(segments[i].errWD().begin(), segments[i].errWD().end(), 0.0, sumsq));
		E2d /= wayptSpc * (std::count_if(segments[i].errWD().begin(), segments[i].errWD().end(), [](double a) {return !std::isnan(a); }) - 1);
		outfile << std::setw(6) << std::fixed << E2d;
		if (i % 4 == 0 ) outfile << "\t";
		else outfile << "\n";
	}
	outfile.close();


	t_print.join();


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