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
	//std::cin >> datafile;
	datafile = "scan5_edgedata.png";

	cv::Mat edges = cv::imread("./Input/" + datafile, cv::IMREAD_GRAYSCALE);

	image = cv::Mat::zeros(raster.size(segments.back().layer()), CV_8UC3);
	raster.draw(image, image, segments.back().layer());
	raster.drawBdry(image, image, segments.back().layer(), cv::Scalar(255, 0, 0), MM2PIX(0.05));
	drawEdges(image, image, edges, cv::Scalar(0, 0, 255), MM2PIX(0.1));
	cv::imwrite(outDir + "edges_" + ".png", image);

	// draw the material
	edgeMsg edgemsg;
	for (int i = 0; i < segments.size(); i++)
	{
		edgemsg.addEdges(edges, i, (i == segments.size() - 1));
		q_edgeMsg.push(edgemsg);
	}

	t_GetMatlErrors(raster, path);

	
	// draw inner and outer edges
	cv::Mat lredges = 255 * cv::Mat::ones(raster.size(), CV_8UC1);
	cv::cvtColor(lredges, lredges, cv::COLOR_GRAY2BGR);
	int thick = MM2PIX(0.1);
	//raster.draw(lredges, lredges, 0, cv::Scalar(0, 0, 0), thick);

	cv::Scalar red(102, 85, 187);
	cv::Scalar blue(136, 68, 0);
	cv::Scalar yel(51, 170, 221);
	cv::Scalar lcolor, rcolor, wpcolor;
	lcolor = cv::Scalar(255,0,0);
	rcolor = cv::Scalar(0, 0, 255);
	wpcolor = cv::Scalar(0, 0, 0);

	for (auto it = segments.begin(); it != segments.end(); ++it) {
		cv::polylines(lredges, (*it).lEdgePts(), false, lcolor, thick);
		cv::polylines(lredges, (*it).rEdgePts(), false, rcolor, thick);
		// draw the waypoints
		for (auto it2 = (*it).waypoints().begin(); it2 != (*it).waypoints().end(); ++it2) {
			cv::circle(lredges, *it2, thick, wpcolor, -1, cv::LINE_AA);
		}
	}
	cv::imwrite(outDir + "LRedges" + ".png", lredges);
	

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