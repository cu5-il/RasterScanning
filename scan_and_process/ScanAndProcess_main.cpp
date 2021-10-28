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
#include "edge_functions.h"

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

//--------------- TRANSLATION ------------------
cv::Mat translateImg(cv::Mat& img, int offsetx, int offsety) {
	cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
	cv::warpAffine(img, img, trans_mat, img.size());
	return img;
}


//========================================================================================================================
int main() {


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//			RASTER
	cv::Mat raster, edgeBoundary;
	std::vector<cv::Point> rasterCoords;
	double border = 1;
	makeRaster(9, 1, border, 1 - 0.04, raster, edgeBoundary, rasterCoords);

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// importing scanned edges
	cv::Mat gblEdges(raster.size(), CV_8U, cv::Scalar({ 0 }));
	readCSV("C:/Users/cu5/Box/Research/Code/Image Processing/Edge Data/gbledges_raster.csv", gblEdges);
	gblEdges.convertTo(gblEdges, CV_8UC1);
//HACK: Shifting glbEdges by a few pixels to the leftso it aligns better
translateImg(gblEdges, -2, 0);

	cv::Mat i_edges = showRaster(raster, gblEdges, cv::Scalar(155, 155, 155));
	
	// Making the edge search regoins
	std::vector<cv::Rect> edgeRegions;
	makeEdgeRegions(rasterCoords, border, edgeRegions);
	//std::vector<cv::Point> lEdgePts, rEdgePts;
	std::vector<std::vector<cv::Point>> lEdgePts, rEdgePts;
	// smooth all the edges
	for (auto it = edgeRegions.begin(); it != edgeRegions.end(); ++it) {
		smoothEdge(*it, gblEdges, lEdgePts, rEdgePts);
	}
	
	int cswitch = 2;
	// Plotting the regions
	cv::Mat i_overlayR = cv::Mat::zeros(raster.size(), CV_8UC3);
	for (auto it = edgeRegions.begin(); it != edgeRegions.end(); ++it) {
		if (cswitch % 2 == 0) {
			cv::rectangle(i_overlayR, *it, cv::Scalar(0, 255, 255), -2);
		}
		else {
			//cv::rectangle(i_edges, *it, cv::Scalar(255, 255, 0), 2);
			cv::rectangle(i_overlayR, *it, cv::Scalar(0, 0, 255), -2);
		}
		cswitch++;
	}
	cv::polylines(i_edges, rasterCoords, false, cv::Scalar(255, 255, 255), 2);
	cv::addWeighted(i_overlayR, 0.5, i_edges, 1, 0, i_edges);

	 //Plotting the smoothed edges
	cswitch = 2;
	for (auto it = lEdgePts.begin(); it != lEdgePts.end(); ++it) {
		if (cswitch % 2 == 0) {
			cv::polylines(i_edges, *it, false, cv::Scalar(0, 255, 255), 1);
		}
		else {
			cv::polylines(i_edges, *it, false, cv::Scalar(255, 255, 0), 1);
		}
		cswitch++;
	}
	cswitch = 2;
	for (auto it = rEdgePts.begin(); it != rEdgePts.end(); ++it) {
		if (cswitch % 2 == 0) {
			cv::polylines(i_edges, *it, false, cv::Scalar(0, 255, 255), 1);
		}
		else {
			cv::polylines(i_edges, *it, false, cv::Scalar(255, 255, 0), 1);
		}
		cswitch++;
	}

	// show the filled regions with edges
	std::vector<cv::Point> aEdgePts;
	cv::Mat i_overlay = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::Mat i_rods = showRaster(raster, gblEdges, cv::Scalar(155, 155, 155));
	for (int i = 0; i < min(lEdgePts.size(), rEdgePts.size()); i++) {
		// drawing the edges
		cv::polylines(i_rods, lEdgePts[i], false, cv::Scalar(255, 255, 0), 1);
		cv::polylines(i_rods, rEdgePts[i], false, cv::Scalar(255, 255, 0), 1);
		// filling the area between the edges
		//aEdgePts.reserve((*itl).size() + (*itr).size()); // preallocate memory
		aEdgePts.insert(aEdgePts.end(), lEdgePts[i].begin(), lEdgePts[i].end());
		aEdgePts.insert(aEdgePts.end(), rEdgePts[i].rbegin(), rEdgePts[i].rend());
		cv::fillPoly(i_overlay, aEdgePts, cv::Scalar(255, 255, 0));
		aEdgePts.clear(); // clear all previous points
	}
	cv::addWeighted(i_overlay, 0.5, i_rods, 1, 0, i_rods);

	// filling the edge area
	cv::Mat i_matl = showRaster(raster, gblEdges, cv::Scalar(155, 155, 155));
	i_overlay = cv::Mat::zeros(raster.size(), CV_8UC3); //resetting the image to 0
	// Combining both edges
	std::vector<cv::Point> allEdgePts;
	allEdgePts.reserve(lEdgePts[0].size() + rEdgePts[0].size()); // preallocate memory
	allEdgePts.insert(allEdgePts.end(), lEdgePts[0].begin(), lEdgePts[0].end());
	allEdgePts.insert(allEdgePts.end(), rEdgePts[0].rbegin(), rEdgePts[0].rend());
	// draw filled region in a separate image
	cv::fillPoly(i_overlay, allEdgePts, cv::Scalar(255, 255, 0));
	double alpha = 0.5;
	cv::addWeighted(i_overlay, alpha, i_matl, 1, 0, i_matl);
	cv::addWeighted(i_overlay, alpha, i_edges, 1, 0, i_edges);

	//// ------------------------------------STUFF FOR SEMINAR-----------------------------------------
	//// Making the scale
	//cv::Point scaleBar(25, scanImg.rows - 25);
	//cv::putText(scanImg, "1mm", scaleBar, cv::FONT_HERSHEY_SIMPLEX, .7, CV_RGB(0, 0, 0), 1, cv::LINE_8);
	//cv::rectangle(scanImg, cv::Rect(scaleBar.x + 6, scaleBar.y + 5, MM2PIX(1), 5), cv::Scalar(0, 0, 0), -1);
	//// Saving the image
	//cv::imwrite("FullScan.png", scanImg);
	////-----------------------------------------------------------------------------------------------

	// Calculate Errors

	std::vector<cv::Point> newCentLine;
	std::vector<cv::Point> newlEdge, newrEdge;

	std::vector <std::vector<double>> errCL, errWD;
	double targetWidth = 0;
	for (int i = 0; i < min(lEdgePts.size(), rEdgePts.size()); i++) {
		std::vector<cv::Point>centerline = { rasterCoords[2 * i], rasterCoords[2 * i + 1] };
		getMatlErrors(centerline, targetWidth, raster.size(), lEdgePts[i], rEdgePts[i], errCL, errWD);
		for (int j = 0; j < centerline.size(); j++) {
			newCentLine.push_back(centerline[j] + cv::Point((int)round(errCL[i][j]), 0));
			newlEdge.push_back(newCentLine.back() - cv::Point((int)round(errWD[i][j] / 2), 0));
			newrEdge.push_back(newCentLine.back() + cv::Point((int)round(errWD[i][j] / 2), 0));
		}
		cv::polylines(i_rods, newCentLine, false, cv::Scalar(0, 0, 255), 1);
		cv::polylines(i_rods, newlEdge, false, cv::Scalar(0, 255, 255), 1);
		cv::polylines(i_rods, newrEdge, false, cv::Scalar(0, 255, 255), 1);
		newCentLine.clear();
		newlEdge.clear();
		newrEdge.clear();
		centerline.clear(); // clear the points in the centerline vector
	}

	addScale(i_rods);

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
