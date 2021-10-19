#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <string>
#include <algorithm>
#include <iterator> 
#include <valarray>

#include <opencv2/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp> // for Kalman filter

#include "A3200.h"

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "scanner_functions.h"
#include "processing_functions.h"
#include "display_functions.h"
#include "makeRaster.h"
#include "gaussianSmooth.h"

// This function will print whatever the latest error was
void PrintError();
void A3200Error(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle);

//using namespace cv;
const char* window_name1 = "Edges";

void writeCSV(std::string filename, cv::Mat m)
{
	std::ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(m, cv::Formatter::FMT_CSV);
	myfile.close();
}

void readCSV(std::string filename, cv::Mat& m)
{
	std::ifstream inFile(filename.c_str());
	std::string single_line;
	std::vector< std::vector<double> > matrix;

	double value;

	if (inFile.is_open()) {
		while (std::getline(inFile, single_line)) {
			std::vector<double> vec;
			std::stringstream temp(single_line);
			std::string single_value;

			while (std::getline(temp, single_value, ',')) {
				value = std::stod(single_value);
				vec.push_back(value);
			}
			matrix.push_back(vec);
		}
	}
	else {
		std::cout << "Unable to open data file: " <<filename<< std::endl;;
		system("pause");
		return;
	}
	m = cv::Mat((int)matrix.size(), (int)matrix[0].size(), CV_64FC1/*, matrix.data()*/);
	m.at<double>(0, 0) = matrix.at(0).at(0);
	for (int i = 0; i < m.rows; ++i)
		for (int j = 0; j < m.cols; ++j)
			m.at<double>(i, j) = matrix.at(i).at(j);
	m = m.clone();
	return;
}

//--------------- SOTRING ------------------
struct myclass {
	bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y); }
} myobject;



//========================================================================================================================
int main() {


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//			RASTER
	cv::Mat raster, edgeBoundary;
	std::vector<cv::Point> rasterCoords;
	
	makeRaster(9, 1, 1, 1 - 0.04, raster, edgeBoundary, rasterCoords);
	

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// importing scanned edges
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	readCSV("C:/Users/cu5/Box/Research/Code/Image Processing/Edge Data/gbledges_raster.csv", gblEdges);
	gblEdges.convertTo(gblEdges, CV_8UC1);
	cv::Mat image = showRaster(raster, gblEdges, false);
	std::vector<cv::Point> gblEdgePts;
	cv::findNonZero(gblEdges, gblEdgePts);
	for (auto it = gblEdgePts.begin(); it != gblEdgePts.end(); ++it) {
		cv::circle(image, *it, 1, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
	}
	

	// TODO: make all the edge regions
	//std::vector<cv::Rect> edgeRegions;
	cv::Rect edgeRegions(rasterCoords[12], rasterCoords[13]+cv::Point(25,0));
	cv::rectangle(image, edgeRegions, cv::Scalar(0,255,0), 1);
	cv::rectangle(image, edgeRegions - cv::Point(25, 0), cv::Scalar(255,0, 255), 1);
	
	std::vector<cv::Point> leftEdgePts, rightEdgePts;
	cv::findNonZero(gblEdges(edgeRegions), rightEdgePts);
	cv::findNonZero(gblEdges(edgeRegions - cv::Point(25, 0)), leftEdgePts);

	for (auto it = leftEdgePts.begin(); it != leftEdgePts.end(); ++it) {
		//*it += cv::Point(25, 50) +edgeRegions.tl();
		*it += (edgeRegions.tl() - cv::Point(25, 0));
	}
	for (auto it = rightEdgePts.begin(); it != rightEdgePts.end(); ++it) {
		*it += edgeRegions.tl();
	}

	//// ------------------------------------STUFF FOR SEMINAR-----------------------------------------
	//// Making an all white image and copying the raster to it
	//cv::Mat scanImg(raster.size(), CV_8UC3, cv::Scalar(255,255,255));
	//cv::polylines(scanImg, rasterCoords, false, cv::Scalar(0, 0, 0),1, cv::LINE_8);

	//// Placing edge points on image
	//for (auto it = gblEdgePts.begin(); it != gblEdgePts.end(); ++it) {
	//	cv::circle(scanImg, *it, 2, CV_RGB(187, 85, 102), -1, cv::LINE_8);
	//}
	//// Making the scale
	//cv::Point scaleBar(25, scanImg.rows - 25);
	//cv::putText(scanImg, "1mm", scaleBar, cv::FONT_HERSHEY_SIMPLEX, .7, CV_RGB(0, 0, 0), 1, cv::LINE_8);
	//cv::rectangle(scanImg, cv::Rect(scaleBar.x + 6, scaleBar.y + 5, MM2PIX(1), 5), cv::Scalar(0, 0, 0), -1);
	//// Saving the image
	//cv::imwrite("FullScan.png", scanImg);
	////-----------------------------------------------------------------------------------------------

	std::cout <<"3rd- point = "<< * next(leftEdgePts.begin(), 2) << std::endl;



	//cv::Mat leftEdgePoints;
	//cv::findNonZero(gblEdges(tallRect), leftEdgePoints);
	//writeCSV("leftEdgePoints.csv", leftEdgePoints);


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//cv::polylines(image, leftEdgePts, false, cv::Scalar(255, 0, 255), 1);
	//cv::polylines(image, rightEdgePts, false, cv::Scalar(0, 0, 255), 1);


	// Sorting the edge points 
	std::sort(rightEdgePts.begin(), rightEdgePts.end(), myobject);
	std::sort(leftEdgePts.begin(), leftEdgePts.end(), myobject);

	// Combining both edges
	std::vector<cv::Point> allEdgePts;
	allEdgePts.reserve(leftEdgePts.size() + rightEdgePts.size()); // preallocate memory
	allEdgePts.insert(allEdgePts.end(), leftEdgePts.begin(), leftEdgePts.end());
	allEdgePts.insert(allEdgePts.end(), rightEdgePts.rbegin(), rightEdgePts.rend());

	// smoothing the edges
	std::vector<cv::Point> rEdgeFilt, lEdgeFilt;
	gaussianSmoothX(leftEdgePts, lEdgeFilt, 7, 3);
	gaussianSmoothX(rightEdgePts, rEdgeFilt, 7, 3);
	cv::polylines(image, rEdgeFilt, false, cv::Scalar(255, 255, 0), 1);
	cv::polylines(image, lEdgeFilt, false, cv::Scalar(255, 255, 0), 1);

	writeCSV("C:/Users/cu5/Box/Research/Code/Image Processing/Edge Data/leftEdgeFilt.csv", cv::Mat(lEdgeFilt));
	writeCSV("C:/Users/cu5/Box/Research/Code/Image Processing/Edge Data/rightEdgeFilt.csv", cv::Mat(rEdgeFilt));
	
	// plotting
	plotEdges(leftEdgePts, lEdgeFilt);
	
	std::cout << "end program " << std::endl;
	//writeCSV("C:/Users/cu5/Box/Research/Code/Image Processing/Edge Data/leftEdgePoints.csv", cv::Mat(leftEdgePts));
	//writeCSV("C:/Users/cu5/Box/Research/Code/Image Processing/Edge Data/rightEdgePoints.csv", cv::Mat(rightEdgePts));
	//writeCSV("C:/Users/cu5/Box/Research/Code/Image Processing/Edge Data/rasterCoords.csv", cv::Mat(rasterCoords));


#ifdef _DEBUG

	system("pause");
#endif
	return 0;
}

//===================================================================
//							FUNCTIONS
//===================================================================

void PrintError() {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	printf("Error : %s\n", data);
}

void A3200Error(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle) {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	printf("Error : %s\n", data);
	system("pause");

	if (NULL != handle) {
		printf("Disconnecting from the A3200.\n");
		if (!A3200Disconnect(handle)) { PrintError(); }
	}
	// Freeing the resources used by the data collection configuration
	if (NULL != DCCHandle) {
		if (!A3200DataCollectionConfigFree(DCCHandle)) { PrintError(); }
	}
}

//{ A3200Error(handle, DCCHandle); return 0; }
