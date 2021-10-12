#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 
#include <string>
#include <algorithm>
#include <iterator> 

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
		std::cout << "Unable to open data file" << std::endl;;
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

struct myclass {
	bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y); }
} myobject;

/*
	define some functions to use as predicates
*/

//Returns true if x is multiple of 10
bool multOf10(int x) {
	return x % 10 == 0;
}

//returns true if item greater than passed in parameter
class Greater {
	int _than;

public:
	Greater(int th) :_than(th) {

	}
	bool operator()(cv::Point point) const
	{
		return point.y > _than;
	}
};

class Less {
	int _than;

public:
	Less(int th) :_than(th) {

	}
	bool operator()(cv::Point point) const
	{
		return point.x < _than;
	}
};


//========================================================================================================================
int main() {

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Loading the raster image
	cv::Mat raster;
	raster = cv::imread("RasterMap9.bmp", cv::IMREAD_COLOR);

	// Making the region around the raster path to search for edges
	cv::Mat edgeBoundary;
	int dilation_size = 20;// was 18 when using 14x14 raster image; 20 seems ok; 23 is max
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));
	cv::dilate(raster, edgeBoundary, element);

	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	readCSV("gbledges_18.20.csv", gblEdges);
	gblEdges.convertTo(gblEdges, CV_8UC1);
	cv::Mat image = showRaster(raster, gblEdges, false);

	std::vector<cv::Point> gblEdgePts;
	cv::findNonZero(gblEdges, gblEdgePts);
	for (auto it = gblEdgePts.begin(); it != gblEdgePts.end(); ++it) {
		cv::circle(image, *it, 1, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
	}
	std::vector<cv::Point> corners = { cv::Point(50, 50), cv::Point(50, 500), cv::Point(100, 500), cv::Point(100, 50) };

	// ------------------------------------STUFF FOR SEMINAR-----------------------------------------
	std::vector<cv::Point> rasterPts;
	for (int i = 0; i < 5; i++) {
		for (auto it = corners.begin(); it != corners.end(); ++it) {
			rasterPts.push_back(*it+i*cv::Point(100, 0));
		}
	}
	

	cv::Mat scanImg(raster.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	//raster.copyTo(scanImg);
	cv::bitwise_not(scanImg, scanImg);
	cv::polylines(scanImg, rasterPts, false, cv::Scalar(0, 0, 0),1, cv::LINE_8);

	// MAking the scale
	cv::Point scaleBar(25, scanImg.rows - 25);
	cv::putText(scanImg, "1mm", scaleBar, cv::FONT_HERSHEY_SIMPLEX, .7, CV_RGB(0, 0, 0), 1, cv::LINE_8);
	cv::rectangle(scanImg, cv::Rect(scaleBar.x + 6, scaleBar.y + 5, 1 / PIX2MM, 5), cv::Scalar(0, 0, 0), -1);

	// Placing edge points on image
	for (auto it = gblEdgePts.begin(); it != gblEdgePts.end(); ++it) {
		cv::circle(scanImg, *it, 2, CV_RGB(187, 85, 102), -1, cv::LINE_8);
	}

	//cv::namedWindow("out", cv::WINDOW_NORMAL);
	//cv::setMouseCallback("out", mouse_callback);
	//cv::imshow("out", scanImg);
	//cv::waitKey(0);
	cv::imwrite("FullScan.png", scanImg);

	return 0;
	//-----------------------------------------------------------------------------------------------

	

	cv::Rect tallRect(corners[0].x-25, corners[0].y, 25, 450);
	cv::Mat gblEdgeROI = gblEdges(tallRect);

	//Look in ROI for edges
	std::vector<cv::Point> leftEdgePts, rightEdgePts;
	cv::findNonZero(gblEdges(tallRect), leftEdgePts);
	cv::findNonZero(gblEdges(tallRect + cv::Point(25, 0)), rightEdgePts);

	for (auto it = leftEdgePts.begin(); it != leftEdgePts.end(); ++it) {
		*it += cv::Point(25, 50);
	}

	for (auto it = rightEdgePts.begin(); it != rightEdgePts.end(); ++it) {
		*it += cv::Point(50, 50);
	}

	std::cout <<"3rd- point = "<< * next(leftEdgePts.begin(), 2) << std::endl;



	//cv::Mat leftEdgePoints;
	//cv::findNonZero(gblEdges(tallRect), leftEdgePoints);
	//writeCSV("leftEdgePoints.csv", leftEdgePoints);


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Kalman filter
	
	float initialEstimateError[2] = { 1e5, 1e5 };;
	float motionNoise[2] = {25, 10};
	float measurementNoise = 25;
	cv::KalmanFilter KF(4, 2, 0);
	std::vector<cv::Point> filteredPts;

	KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 
		1, 1, 0, 0,  
		0, 1, 0, 0,  
		0, 0, 1, 1,  
		0, 0, 0, 1);				// A
	KF.measurementMatrix = (cv::Mat_<float>(2, 4) <<
		1, 0, 0, 0,
		0, 0, 1, 0);				// H
	KF.errorCovPost = (cv::Mat_<float>(4, 4) <<
		initialEstimateError[0], 0, 0, 0,
		0, initialEstimateError[1], 0, 0,
		0, 0, initialEstimateError[0], 0,
		0, 0, 0, initialEstimateError[1]);			// P(k)
	KF.processNoiseCov = (cv::Mat_<float>(4, 4) << 
		motionNoise[0], 0, 0, 0,
		0, motionNoise[1], 0, 0,
		0, 0, motionNoise[0], 0,
		0, 0, 0, motionNoise[1]);	// Q 
	KF.measurementNoiseCov = (cv::Mat_<float>(4, 4) << 
		measurementNoise, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, measurementNoise, 0,
		0, 0, 0, 0);			// R Error in measurement?
	

	cv::Mat_<float> measurement(4, 1); measurement.setTo(cv::Scalar(0));


	KF.statePre.at<float>(0) = leftEdgePts[0].x;
	KF.statePre.at<float>(1) = 0;
	KF.statePre.at<float>(2) = leftEdgePts[0].y;
	KF.statePre.at<float>(3) = 0;

	//for (auto it = next(leftEdgePts.begin(),1); it != leftEdgePts.end(); ++it) {
	//	// First predict, to update the internal statePre variable
	//	cv::Mat prediction = KF.predict();
	//	std::cout << "prediction = \n" << prediction << std::endl;
	//	measurement(0) = (*it).x;
	//	measurement(1) = (*it).y;
	//	std::cout << "measurement = \n" << measurement << std::endl;
	//	cv::Mat estimated = KF.correct(measurement);
	//	std::cout << "estimated = \n" << estimated << std::endl << std::endl;
	//	filteredPts.push_back(cv::Point((int)estimated.at<float>(0), (int)estimated.at<float>(1)));
	//}
	//cv::polylines(image, filteredPts, false, cv::Scalar(255, 0, 255), 1);


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//cv::polylines(image, leftEdgePts, false, cv::Scalar(0, 0, 255), 1);
	//cv::polylines(image, rightEdgePts, false, cv::Scalar(0, 0, 255), 1);
	//cv::rectangle(image, tallRect, cv::Scalar(0, 255, 0), 1);
	cv::namedWindow("out", cv::WINDOW_NORMAL);
	cv::setMouseCallback("out", mouse_callback);
	cv::imshow("out", image);

	//std::sort(corners.begin(), corners.end(), myobject);



	cv::waitKey(0);
	//========================================================================================================================


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
