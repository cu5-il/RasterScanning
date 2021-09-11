#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 

#include <opencv2/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "A3200.h"

//#include "scanner_functions.h"
#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"

typedef struct {
	double x;
	double y;
	double z;
	double T;
}Coords;


// This function will print whatever the latest error was
//void PrintError();
using namespace cv;

Mat callbackImg, callbackImg2;
std::vector<Point> callbackPts;
const char* window_name1 = "Edges";
int lowEdgeThresh = 10;
int highEdgeThresh = 30;

void processData(cv::Mat data, double collectedData[][NUM_DATA_SAMPLES], Coords fbk, cv::Mat profile);

void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == EVENT_LBUTTONDOWN) {
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
}


int main() {

	//A3200Handle handle = NULL;
	//A3200DataCollectConfigHandle DCCHandle = NULL;
	double collectedData[NUM_DATA_SIGNALS][NUM_DATA_SAMPLES];
	cv::Mat Data(NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, CV_64F);
	//double X_fb, Y_fb, Z_fb, T_fb;

	// INITIALIZATION & Data collection
	//========================================================================================================================
	/*
	//Connecting to the A3200
	std::cout << "Connecting to A3200. Initializing if necessary.\n";
	if (!A3200Connect(&handle)) { PrintError(); goto cleanup; }
	// Creating a data collection handle
	if (!A3200DataCollectionConfigCreate(handle, &DCCHandle)) { PrintError(); goto cleanup; }
	// Setting up the data collection
	if (!setupDataCollection(handle, DCCHandle)) { PrintError(); goto cleanup; }

	// Collecting the data
	std::cout << "Starting data collection\n";
	if (collectData(handle, DCCHandle, &collectedData[0][0])) {
		// Copy the data to a matrix
		std::memcpy(Data.data, collectedData, NUM_DATA_SIGNALS * NUM_DATA_SAMPLES * sizeof(double));
	}
	else{ PrintError(); goto cleanup; }
	*/

	// Faking the data
	std::ifstream inFile("rawData.txt");
	if (inFile.is_open()) {
		for (int i = 0; i < NUM_DATA_SIGNALS; i++) {
			for (int j = 0; j < NUM_DATA_SAMPLES; j++) {
				inFile >> collectedData[i][j];
			}
		}
		inFile.close();
	}
	else {
		std::cout << "Unable to open data file" << std::endl;;
		system("pause");
		return(0);
	}
	std::memcpy(Data.data, collectedData, NUM_DATA_SIGNALS * NUM_DATA_SAMPLES * sizeof(double));
	//========================================================================================================================

	int fbIdx[2];
	int profileIdx;
	double xfb, yfb, /*zfb,*/ Tfb;

	// Get the position feedback when the laser was triggered
	cv::minMaxIdx(Data.row(1), NULL, NULL, fbIdx, NULL);
	std::cout << "Min value is at index = " << fbIdx[1] << std::endl;


	// Finding the voltage header of the scanner signal
	cv::Mat Z_V;
	cv::Mat Z_V_edge;
	cv::Mat Z_V_edgeIdx;

	cv::normalize(Data(cv::Range(0, 1), cv::Range(fbIdx[1], Data.cols)), Z_V, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::Canny(Z_V, Z_V_edge, 10, 30, 3);
	cv::findNonZero(Z_V_edge, Z_V_edgeIdx);

	profileIdx = Z_V_edgeIdx.at<int>(1, 0) + fbIdx[1];
	std::cout << "Profile start @ " << profileIdx << std::endl;

	// Isolating the scanned profile
	cv::Mat Z = Data(cv::Rect(Z_V_edgeIdx.at<int>(1, 0) + fbIdx[1], 0, NUM_PROFILE_PTS, 1)) / OPAMP_GAIN;

	// Interpolate Z so it is the same scale as the raster reference image
	cv::resize(Z, Z, Size(std::round(SCAN_WIDTH / PIX2MM), Z.rows), INTER_LINEAR);
	cv::Mat Z_img;
	cv::normalize(Z, Z_img, 0, 255, cv::NORM_MINMAX, CV_8U);

	// Calculating the X & Y coordinates of the scan
	// --------------------------------------------------------------------------
	std::cout << "Zcols = " << Z.cols << " Math = " << std::round(SCAN_WIDTH / PIX2MM) << std::endl;

	std::vector<double> X(std::round(SCAN_WIDTH / PIX2MM), 0);
	std::vector<double> Y(std::round(SCAN_WIDTH / PIX2MM), 0);

	std::vector<double> printROI = { -1, -1, 13, 13 }; //IMPORT FROM FILE

	double R = 5;
	xfb = 10;
	yfb = 9.8;
	Tfb = 0;
	double Zx;
	cv::Mat scanMask(Z.size(), CV_8UC1, Scalar(255));
	int startIdx = -1, endIdx = 0;

	for (int i = 0; i < Z.cols; i++) {
		// Local coordinate of the scanned point
		Zx = -SCAN_WIDTH / 2 + i * SCAN_WIDTH / ((int)Z.cols - 1);
		// Transforming local coordinate to global coordinate
		X[i] = xfb - R * cos(Tfb * PI / 180) - Zx * sin(Tfb * PI / 180);
		Y[i] = yfb - R * sin(Tfb * PI / 180) + Zx * cos(Tfb * PI / 180);
		// Check if scanned point in outside the print ROI 
		if ((X[i] < printROI[0] || printROI[2] < X[i] || Y[i] < printROI[1] || printROI[3] < Y[i])) {
			scanMask.at<uchar>(0, i) = 0;
		}
		else if (startIdx == -1) {
			startIdx = i;
		}
		else { endIdx = i; }
	}
	//convert the start and end (X,Y) coordinates of the scan to points on the image
	cv::Point start(std::round((Y[startIdx] - printROI[1]) / PIX2MM), std::round((X[startIdx] - printROI[0]) / PIX2MM));
	cv::Point end(std::round((Y[endIdx] - printROI[1]) / PIX2MM), std::round((X[endIdx] - printROI[0]) / PIX2MM));

	cv::Mat Zroi = Z.colRange(Range(startIdx, endIdx));
	cv::Mat Zroi_img;
	cv::normalize(Zroi, Zroi_img, 0, 255, cv::NORM_MINMAX, CV_8U);

	cv::Mat maskedProfile;
	cv::bitwise_and(Z_img, scanMask, maskedProfile);

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Loading the raster image
	cv::Mat raster, rasterMask;
	raster = cv::imread("RasterMap.bmp", IMREAD_COLOR);
	cv::Mat rasterColor(raster.size(), CV_8UC3, Scalar({ 0, 255, 0, 0 }));
	bitwise_not(raster, rasterMask);
	raster.copyTo(rasterColor, rasterMask);

	// Stretching the profile
	cv::Mat Z_img_tall;
	int height = 40;
	cv::resize(Zroi_img, Z_img_tall, Size(Zroi_img.cols, height), INTER_LINEAR);
	cvtColor(Z_img_tall, Z_img_tall, COLOR_GRAY2BGR);

	// Copy the profile to the raster
	Z_img_tall.copyTo(rasterColor(cv::Rect(start, Z_img_tall.size())), rasterMask(cv::Rect(start, Z_img_tall.size())));


	cv::line(rasterColor, start, end, Scalar(0, 0, 255), 2); // draw line where scan was taken

	//cv::namedWindow("Overlay", cv::WINDOW_AUTOSIZE); 
	//cv::setMouseCallback("Overlay", mouse_callback);
	//cv::imshow("Overlay", rasterColor); 


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Making the region around the raster path to search for edges
	Mat dilation_dst;
	int dilation_size = 18;// 13 wa sa little too small; 20 seems ok; 23 is max
	Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	dilate(raster, dilation_dst, element);

	//cv::namedWindow("Search Mask", cv::WINDOW_AUTOSIZE);
	//cv::setMouseCallback("Search Mask", mouse_callback);
	//cv::imshow("Search Mask", dilation_dst);

	// Using a line iterator to extract the points along the scan line
	cv::LineIterator Lit(dilation_dst, start, end);

	std::vector<Point> points;
	points.reserve(Lit.count);
	for (int i = 0; i < Lit.count; ++i, ++Lit) {
		points.push_back(Lit.pos());
	}
	//std::cout <<"line coords "<< points << std::endl;

	// Alternative line iteration
	//https://docs.opencv.org/4.5.1/dc/dd2/classcv_1_1LineIterator.html
	LineIterator it(dilation_dst, start, end, 8);
	LineIterator it2 = it;
	std::vector<uchar> buf(it.count);
	std::vector<Point> points2;
	points2.reserve(it2.count);
	uchar lastVal = 0;
	uchar curVal = 0;
	// find the edges of the dialeted raster 
	for (int i = 0; i < it.count; i++, ++it) {
		//	buf[i] = *(const uchar*)*it;
		curVal = *(const uchar*)*it;
		if ((curVal == 255) && (lastVal == 0) && (i != 0)) { // find rising edges
			points2.push_back(it.pos());
		}
		if ((curVal == 0) && (lastVal == 255) && (i != 0)) { // find falling edges
			points2.push_back(it.pos());
		}
		lastVal = curVal;
	}
	if ((points2.size() % 2) != 0) {
		std::cout << "ERROR: odd number of edges" << std::endl;
		return 0;
	}
	std::cout << "Search from:" << std::endl;
	//-------------------------------------------------------------------------------------------------------------
	// Search within the edges of the dialated raster for the actual edges
	cv::Mat searchROI;
	cv::Mat ROIedges;
	std::vector<Point> ROIedgeIdx;
	cvtColor(Zroi_img, Zroi_img, COLOR_GRAY2BGR); //convert global image to color
	for (int i = 0; i < points2.size(); i = i + 2) {

		std::cout << "-------------" << std::endl;
		std::cout << points2[i].x << " to " << points2[i + 1].x << std::endl;
		searchROI = Zroi(cv::Range::all(), cv::Range(points2[i].x, points2[i + 1].x)); // isolate single peak
		cv::normalize(searchROI, searchROI, 0, 255, cv::NORM_MINMAX, CV_8U);
		cv::Canny(searchROI, ROIedges, 10, 20, 7);

		findNonZero(ROIedges, ROIedgeIdx);

		cv::Vec3b edgeColor;
		if (ROIedgeIdx.size() == 2) { //verify that only two edges were found
			edgeColor = Vec3b(0, 0, 255);
		}
		else {
			//edgeColor = Vec3b(0, 128, 255); //orange
			edgeColor = Vec3b(255, 0, 188); //purple
		}

		//mark search regions on global image
		Zroi_img.at<Vec3b>(Point(points2[i].x, 0)) = Vec3b(255, 0, 0);
		Zroi_img.at<Vec3b>(Point(points2[i + 1].x, 0)) = Vec3b(255, 0, 0);

		for (int j = 0; j < ROIedgeIdx.size(); j++) {
			Zroi_img.at<Vec3b>(Point(ROIedgeIdx[j].x + points2[i].x, 0)) = edgeColor;
		}
		std::cout << "Num edges = " << ROIedgeIdx.size() << std::endl;
		std::cout << ROIedgeIdx << std::endl;

		//cv::namedWindow("ROI", cv::WINDOW_NORMAL);
		//cv::setMouseCallback("ROI", mouse_callback);
		//cv::imshow("ROI", searchROI);
		//cv::namedWindow("ROI edges", cv::WINDOW_NORMAL);
		//cv::setMouseCallback("ROI edges", mouse_callback);
		//cv::imshow("ROI edges", ROIedges);
		//waitKey(1);
	}
	std::cout << "-------------" << std::endl;

	cv::namedWindow("Global ROI", cv::WINDOW_NORMAL);
	cv::setMouseCallback("Global ROI", mouse_callback);
	cv::imshow("Global ROI", Zroi_img);



	cv::waitKey(0);
	return 0;
	//-----------------------------------------------------------------------------------------------------------------
// stuff for callback~~~~~~~~~~~~~~~~
	cvtColor(Zroi_img, callbackImg, COLOR_GRAY2BGR); //convert global image to color
	callbackPts = points2;
	namedWindow(window_name1, cv::WINDOW_NORMAL);

	// create a toolbar
	createTrackbar("Low", window_name1, &lowEdgeThresh, 100, changeEdgeThresh);
	createTrackbar("High", window_name1, &highEdgeThresh, 300, changeEdgeThresh);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



		// Making a image to store all the global information
	Mat globalImg(raster.size(), CV_8UC3, Scalar({ 0, 0, 0, 0 }));

	Z_img_tall.copyTo(globalImg(cv::Rect(start, Z_img_tall.size())), dilation_dst(cv::Rect(start, Z_img_tall.size())));

	Mat globalEdge;
	cv::Canny(globalImg, globalEdge, 10, 30, 3);
	Mat red(raster.size(), CV_8UC3, Scalar({ 0, 0, 255, 0 }));
	red.copyTo(globalImg, globalEdge);


	//cv::namedWindow("global", cv::WINDOW_AUTOSIZE); // Create a window for display.
	//cv::setMouseCallback("global", mouse_callback);
	//cv::imshow("global", globalImg); // Show our image inside it.

	cv::waitKey(0); // Wait for a keystroke in the window

	//========================================================================================================================
//cleanup:
//
//	if (NULL != handle) {
//		printf("Disconnecting from the A3200.\n");
//		if (!A3200Disconnect(handle)) { PrintError(); }
//	}
//	// Freeing the resources used by the data collection configuration
//	if (NULL != DCCHandle) {
//		if (!A3200DataCollectionConfigFree(DCCHandle)) { PrintError(); }
//	}

#ifdef _DEBUG
	system("pause");
#endif
	return 0;
}

//===================================================================
//							FUNCTIONS
//===================================================================

//void PrintError() {
//	CHAR data[1024];
//	A3200GetLastErrorString(data, 1024);
//	printf("Error : %s\n", data);
//}



/// @brief Processes the raw data signals.
/// 
/// 
/// @param[in]	data	array of signals where the rows are: [0] analog scanner output, [1] triggering signal, [2] x, [3] y, [4] z, and [5] theta position
/// @param[out]	fbk	x, y, z, and theta coordinates of gantry when scan was taken
/// @param[out] profile	Z profile from scanner
/// @param[out] 
///
void processData(cv::Mat data, double collectedData[][NUM_DATA_SAMPLES], Coords fbk, cv::Mat profile) {
	int fbIdx[2];
	int profileStartIdx;
	double xfb, yfb, /*zfb,*/ Tfb;
	cv::Mat dataMat;
	cv::Mat profileVoltage_8U, profileEdges, profileEdgesIdx;


	// copying the collected data into a matrix
	std::memcpy(dataMat.data, collectedData, NUM_DATA_SIGNALS * NUM_DATA_SAMPLES * sizeof(double));

	// Get the position feedback when the laser was triggered
	cv::minMaxIdx(data.row(1), NULL, NULL, fbIdx, NULL); //find the rising edge of the trigger signal sent to the laser
	fbk.x = data.at<double>(2, fbIdx[1]); // assigning the position feedback values
	fbk.y = data.at<double>(3, fbIdx[1]);
	fbk.z = data.at<double>(4, fbIdx[1]);
	fbk.T = data.at<double>(5, fbIdx[1]);

	// Finding the voltage header of the scanner signal, i.e find the start of the scanned profile
	// Search only the data after the triggering signal was sent (after index fbIdx[1])
	cv::normalize(data(cv::Range(0, 1), cv::Range(fbIdx[1], data.cols)), profileVoltage_8U, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::Canny(profileVoltage_8U, profileEdges, 10, 30, 3);
	cv::findNonZero(profileEdges, profileEdgesIdx);

	profileStartIdx = profileEdgesIdx.at<int>(1, 0) + fbIdx[1]; // Starting index of the scan profile
	profile = data(cv::Rect(profileStartIdx, 0, NUM_PROFILE_PTS, 1)) / OPAMP_GAIN; // Isolating the scanned profile and converting to height

	// Interpolate Z so it is the same scale as the raster reference image
	//cv::resize(Z, Z, Size(std::round(SCAN_WIDTH / PIX2MM), Z.rows), INTER_LINEAR);
	//cv::Mat Z_img;
	//cv::normalize(Z, Z_img, 0, 255, cv::NORM_MINMAX, CV_8U);
	return;
}

/*
void foo(cv::Mat scan, double ROIcoords[4], int &startIdx, int &endIdx) {

	for (int i = 0; i < Z.cols; i++) {
		// Local coordinate of the scanned point
		Zx = -SCAN_WIDTH / 2 + i * SCAN_WIDTH / ((int)Z.cols - 1);
		// Transforming local coordinate to global coordinate
		X[i] = xfb - R * cos(Tfb * PI / 180) - Zx * sin(Tfb * PI / 180);
		Y[i] = yfb - R * sin(Tfb * PI / 180) + Zx * cos(Tfb * PI / 180);
		// Check if scanned point in outside the print ROI
		if ((X[i] < printROI[0] || printROI[2] < X[i] || Y[i] < printROI[1] || printROI[3] < Y[i])) {
			scanMask.at<uchar>(0, i) = 0;
		}
		else if (startIdx == -1) {
			startIdx = i;
		}
		else { endIdx = i; }
	}


	return 0;
}
*/
