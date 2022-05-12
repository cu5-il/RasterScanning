#include <iostream>
#include <fstream>
#include <iomanip>      // std::setw
#include <vector> 
#include <string>
#include <numeric> // std::accumulate

#include <deque>
#include <filesystem> // std::filesystem::copy, std::filesystem::create_directories
namespace fs = std::filesystem;

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
#include "input.h"
#include "multiLayer.h"

std::string datetime(std::string format = "%Y.%m.%d-%H.%M.%S");

int main() {
	// Disable openCV warning in console
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
	// Create and set the output path
	outDir.append(datetime("%Y.%m.%d") + "/");
	try { fs::create_directories(outDir); }
	catch (std::exception& e) { std::cout << e.what(); }

	std::thread t_scan, t_process, t_control, t_print;

	// ---------------------------- LOADING INPUTS ----------------------------
	// Initialize parameters
	Raster raster;
	double rasterBorder = 2;
	std::vector<std::vector<Path>> path, ctrlPath;

	// defining the material models
	MaterialModel augerModel = MaterialModel(MaterialModel::AUGER,
		std::vector<double>{1.0, 1.5, 2.0, 2.5, 3.0},
		std::vector<double>{1.5259, 1.1374, 0.93121, 0.7236, 0.59139},
		std::vector<double>{0.8, 0.8, 0.8, 0.8, 0.8},
		std::vector<double>{-0.10408, -0.073806, -0.050416, 0.017425, 0.056501});

	MaterialModel velocityModel = MaterialModel(MaterialModel::VELOCITY,
		std::vector<double>{0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
		std::vector<double>{0.3797, 0.52605, 0.67591, 0.81265, 0.95039, 1.0902, 1.2048, 1.3226},
		std::vector<double>{-0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8},
		std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

	MaterialModel matModel;

	// setting the print options
	double leadin = 10;
	PrintOptions printOpts(leadin);
	printOpts.extrude = true;
	printOpts.disposal = false;
	printOpts.asyncTheta = 80;// 32;

	// Getting user input
	std::string resp, infile;
	char option;
	int lineNum;
	infile = "./Input/printTable.md";

	std::cout << "Select option: (p)rint, (s)can, or print (w)ithout control? ";
	std::cin >> option;
	std::cout << "Test #: ";
	std::cin >> lineNum;

	// Reading the input parameters
	TableInput input(infile, lineNum);
	// copy the table to the output folder
	try { fs::copy(infile, outDir, fs::copy_options::overwrite_existing); }
	catch (std::exception& e) { std::cout << e.what(); }

	// Append the output directory with the print number and create the directory
	outDir.append("/print" + std::to_string(lineNum) + "/");
	try { fs::create_directories(outDir); }
	catch (std::exception& e) { std::cout << e.what(); }

	// Setting the control method (auger or velocity)
	switch (input.method)
	{
	default:
		std::cout << "ERROR: Unknown control type.\n";
		system("pause");
		return 0;
		break;
	case MaterialModel::AUGER:
		matModel = augerModel;
		break;
	case MaterialModel::VELOCITY:
		matModel = velocityModel;
		break;
	}
	
	// Setting up the controller
	PController controller(matModel);
	controller.setAugerLimits(0.3, 1.5);
	controller.setFeedLimits(0.5, 4.0);
	
	// make the scaffold
	raster = Raster(input.length, input.width, input.rodSpc, input.rodSpc - .1, rasterBorder);
	raster.offset(cv::Point2d(input.initPos.x, input.initPos.y));
	//MultiLayerScaffold scaffold(input, raster);
	FunGenScaf scaffold(input, raster, matModel);
	// add a lead out line
	printOpts.leadout = -SCAN_OFFSET_X + 1;
	scaffold.leadout(-SCAN_OFFSET_X);

	path = scaffold.path;
	segments = scaffold.segments;

	// Load the inputs for the initial segments
	if (option == 'p')
	{
		std::string filename, line;
		std::cout << "Enter file name containg initial segment inputs or press ENTER to continue\n";
		std::cin.ignore();
		std::getline(std::cin, filename);
		if (!filename.empty())
		{
			std::ifstream inFile("./Input/" + filename + ".txt");
			std::vector<double> f, e;
			if (inFile.is_open())
			{
				for (int i = 0; i < 3; i += 2)
				{
					// read the segment inputs
					std::getline(inFile, line);
					std::stringstream ss(line);
					f = std::vector<double>(std::istream_iterator<double>(ss), std::istream_iterator<double>());
					std::getline(inFile, line);
					ss = std::stringstream(line);
					e = std::vector<double>(std::istream_iterator<double>(ss), std::istream_iterator<double>());
					// assign the segment inputs
					for (int j = 0; j < path[i].size(); j++) {
						path[i][j].f = f[j];
						path[i][j].e = e[j];
					}
				}
				inFile.close();
			}
			else {
				std::cout << "Unable to open: " << filename << std::endl;
				system("pause");
				return 0;
			}
		}
	}

	cv::Mat imseg = raster.draw(input.startLayer);
	drawSegments(raster.draw(input.startLayer), imseg, segments, raster.origin(), input.startLayer, 3);
	cv::Mat image = cv::Mat::zeros(raster.size(segments.back().layer()), CV_8UC3);
	drawMaterial(image, image, scaffold.segments, scaffold.path, scaffold.segments.back().layer());

	// ---------------------------- PRINTING & SCANNING ----------------------------

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

#ifdef DEBUG_SCANNING
	q_scanMsg.push(true);
	t_CollectScans(raster);
	return 0;
#endif // DEBUG_SCANNING

	switch (option)
	{
	default:
		return 0;
	case 'p': // PRINTING
		outDir.append("print_");
		ctrlPath = path;

		t_scan = std::thread{ t_CollectScans, raster };
		t_process = std::thread{ t_GetMatlErrors, raster, path };
		t_print = std::thread{ t_printQueue, path[0][0], printOpts };
		t_control = std::thread{ t_controller, std::ref(ctrlPath), std::ref(controller) };

		t_scan.join();
		t_process.join();
		t_control.join();

		break;
	case 'w': // PRINTING WITHOUT CONTROL
		outDir.append("printNC_");
		ctrlPath = path;

		t_scan = std::thread{ t_CollectScans, raster };
		t_process = std::thread{ t_GetMatlErrors, raster, path };
		t_print = std::thread{ t_printQueue, path[0][0], printOpts };
		t_control = std::thread{ t_noController, ctrlPath };

		t_scan.join();
		t_process.join();
		t_control.join();

		break;

	case 's': // SCANNING
		Raster rasterScan;
		segments.clear();
		path.clear();
		printOpts.extrude = false;
		std::cout << "(r)otating or (f)ixed scan: ";
		std::cin >> option;
		switch (option)
		{
		default:
			return 0;
		case 'r':
			outDir.append("scanR_");
			rasterScan = raster;
			if (input.startLayer % 2 == 0) { rasterScan.offset(cv::Point2d(0, 1.5)); } // offset path in y direction
			else { rasterScan.offset(cv::Point2d(1.5, 0)); } // offset path in x direction
			input.initPos += cv::Point3d(0, 0, 1); // raise path
			scaffold = FunGenScaf (input, rasterScan, augerModel);
			scaffold.leadout(-SCAN_OFFSET_X);
			segments = scaffold.segments;
			path = scaffold.path;
			ctrlPath = scaffold.path;

			t_scan = std::thread{ t_CollectScans, raster };
			t_process = std::thread{ t_GetMatlErrors, raster, path };
			t_print = std::thread{ t_printQueue, path[0][0], printOpts };
			t_control = std::thread{ t_controller, std::ref(ctrlPath), std::ref(controller) };

			t_scan.join();
			t_process.join();
			t_control.join();
			break;
		case 'f':
			outDir.append("scanF_");
			printOpts.leadin = 0;
			printOpts.leadout = 0;
			printOpts.asyncTheta = 0;
			
			segments = scaffold.segmentsScan;
			path = scaffold.pathScan;
			ctrlPath = scaffold.pathScan;
			t_scan = std::thread{ t_CollectScans, raster };
			t_print = std::thread{ t_printQueue, path[0][0], printOpts };
			t_control = std::thread{ t_noController, ctrlPath };

			t_scan.join();
			t_control.join();

			// Calculate the errors using the actual rater path used in the print
			segments.clear();
			path.clear();
			path = scaffold.path;
			ctrlPath = scaffold.path;
			segments = scaffold.segments;

			// Purge all existing edge messages
			edgeMsg edgemsg;
			while (!q_edgeMsg.empty()) { q_edgeMsg.wait_and_pop(edgemsg); }
			cv::Mat edges = edgemsg.edges();
			// Load the new edge messages
			for (int i = 0; i < segments.size(); i++) 
			{ 
				edgemsg.addEdges(edges, i, (i == segments.size() - 1)); 
				q_edgeMsg.push(edgemsg);
			}

			t_GetMatlErrors(raster, path);
			break;
		}

		break;
	}

	// ---------------------------- SAVING DATA ----------------------------

	// Opening a file to save the results
	std::ofstream outfile;
	outfile.open(std::string(outDir + "pathData.txt").c_str());
	outfile.precision(3);
	// loop through each long segment
	for (int i = 0; i < path.size(); i += 2) {
		// loop through all the waypoints
		for (int j = 0; j < path[i].size(); j++) {
			outfile << std::setw(3) << std::fixed << i << "\t";// segment number
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

	// Use the errors from the last segments to calcualte the inputs for the next layer and save them to a txt file
	if (option == 'p')
	{
		outfile.open(std::string(outDir + "nextLayerInputs.txt").c_str());
		errsMsg errMsg;

		while (!q_errsMsg.empty()) {
			q_errsMsg.wait_and_pop(errMsg);
			if (!errMsg.errCL().empty() && !errMsg.errWD().empty()) {
				Path nextPath;
				std::vector<double> f, e;
				// calculate the inputs
				for (int i = 0; i < errMsg.errWD().size(); i++) {
					controller.nextPath(nextPath, ctrlPath[errMsg.segmentNum()][i], errMsg.errWD()[i], errMsg.errCL()[i]);
					f.push_back(nextPath.f);
					e.push_back(nextPath.e);
				}
				// write the inputs to the txt file
				for (auto const& v : f) outfile << v << '\t';
				outfile << '\n';
				for (auto const& v : e) outfile << v << '\t';
			}
		}
		outfile.close();
	}

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
		E2d /= input.wayptSpc * (std::count_if(segments[i].errWD().begin(), segments[i].errWD().end(), [](double a) {return !std::isnan(a); }) - 1);
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