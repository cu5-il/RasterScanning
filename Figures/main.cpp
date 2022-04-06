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

	// Initialize parameters
	Raster raster;
	double rasterBorder = 1;
	std::vector<std::vector<Path>> path, ctrlPath;

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
	std::string datafile, infile;
	int lineNum;
	infile = "./Input/printTable.md";

	std::cout << "Enter Test #: ";
	std::cin >> lineNum;


	TableInput input(infile, lineNum);
	// copy the table to the output folder
	try { fs::copy(infile, outDir, fs::copy_options::overwrite_existing); }
	catch (std::exception& e) { std::cout << e.what(); }

	// Append the output directory with the print number and create the directory
	outDir.append("/print" + std::to_string(lineNum) + "/");
	try { fs::create_directories(outDir); }
	catch (std::exception& e) { std::cout << e.what(); }

	// make the scaffold
	raster = Raster(input.length, input.width, input.rodSpc, input.rodSpc - .1, rasterBorder);
	raster.offset(cv::Point2d(input.initPos.x, input.initPos.y));
	//MultiLayerScaffold scaffold(input, raster);
	FunGenScaf scaffold(input, raster, augerModel);

	path = scaffold.path;
	segments = scaffold.segments;

	// draw the segments
	cv::Mat imseg = raster.draw(input.startLayer);
	drawSegments(raster.draw(input.startLayer), imseg, segments, raster.origin(), input.startLayer, 3);
	cv::Mat image = cv::Mat::zeros(raster.size(segments.back().layer()), CV_8UC3);
	drawMaterialSegments(image, image, scaffold.segments, scaffold.path, scaffold.segments.back().layer());

	std::cout << "Enter file to load ";
	std::cin >> datafile;

	cv::Mat edges = cv::imread("./Input/" + datafile, cv::IMREAD_GRAYSCALE);

	// draw the material
	edgeMsg edgemsg;
	for (int i = 0; i < segments.size(); i++)
	{
		edgemsg.addEdges(edges, i, (i == segments.size() - 1));
		q_edgeMsg.push(edgemsg);
	}

	t_GetMatlErrors(raster, path);

	/*
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
		E2d /= input.wayptSpc * (std::count_if(segments[i].errWD().begin(), segments[i].errWD().end(), [](double a) {return !std::isnan(a); }) - 1);
		outfile << std::setw(6) << std::fixed << E2d;
		if (i % 4 == 0) outfile << "\t";
		else outfile << "\n";
	}
	outfile.close();
	*/

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