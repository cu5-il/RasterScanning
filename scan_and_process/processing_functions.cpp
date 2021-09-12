#include <iostream>
#include <fstream>
#include <cmath>
#include <vector> 

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


void processData(double data[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& profile) {
	int fbIdx[2];
	int profileStartIdx;
	cv::Mat profileVoltage_8U, profileEdges, profileEdgesIdx;

	cv::Mat dataMat(NUM_PROFILE_PTS, NUM_DATA_SAMPLES, CV_64F, data); // copying the collected data into a matrix
	
	// Get the position feedback when the laser was triggered
	cv::minMaxIdx(dataMat.row(1), NULL, NULL, fbIdx, NULL); //find the rising edge of the trigger signal sent to the laser
	fbk->x = dataMat.at<double>(2, fbIdx[1]); // assigning the position feedback values
	fbk->y = dataMat.at<double>(3, fbIdx[1]);
	fbk->z = dataMat.at<double>(4, fbIdx[1]);
	fbk->T = dataMat.at<double>(5, fbIdx[1]);

	// Finding the voltage header of the scanner signal, i.e find the start of the scanned profile
	// Search only the data after the triggering signal was sent (after index fbIdx[1])
	cv::normalize(dataMat(cv::Range(0, 1), cv::Range(fbIdx[1], dataMat.cols)), profileVoltage_8U, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::Canny(profileVoltage_8U, profileEdges, 10, 30, 3);
	cv::findNonZero(profileEdges, profileEdgesIdx);

	profileStartIdx = profileEdgesIdx.at<int>(1, 0) + fbIdx[1]; // Starting index of the scan profile
	profile = dataMat(cv::Rect(profileStartIdx, 0, NUM_PROFILE_PTS, 1)).clone() / OPAMP_GAIN; // Isolating the scanned profile and converting to height

	return;
}

void local2globalScan(int profileCols, const Coords fbk, const std::vector<double>& printROI, cv::Point &profileStart, cv::Point& profileEnd , cv::Range& profileROIRange) {
	std::cout << "print roi = ";
	for (auto i : printROI)
		std::cout << i << ", ";
	std::cout << std::endl;
	std::vector<double> XY_start(2),XY_end(2);
	double X, Y;
	double R = SCAN_OFFSET;
	double local_x;
	int startIdx = -1, endIdx = 0;

	for (int i = 0; i < profileCols; i++) {
		// Local coordinate of the scanned point
		local_x = -SCAN_WIDTH / 2 + i * SCAN_WIDTH / (profileCols - 1);
		// Transforming local coordinate to global coordinate
		X = fbk.x - R * cos(fbk.T * PI / 180) - local_x * sin(fbk.T * PI / 180);
		Y = fbk.y - R * sin(fbk.T * PI / 180) + local_x * cos(fbk.T * PI / 180);
		// Check if scanned point in outside the print ROI 
		if ((X < printROI[0] || printROI[2] < X || Y < printROI[1] || printROI[3] < Y)) {
		}
		else if (startIdx == -1) {
			startIdx = i;
			XY_start = { X, Y };
		}
		else { 
			endIdx = i; 
			XY_end = { X, Y };
		}
	}
	//convert the start and end (X,Y) coordinates of the scan to points on the image
	cv::Point startPx(std::round((XY_start[1] - printROI[1]) / PIX2MM), std::round((XY_start[0] - printROI[0]) / PIX2MM));
	cv::Point endPx(std::round((XY_end[1] - printROI[1]) / PIX2MM), std::round((XY_end[0] - printROI[0]) / PIX2MM));

	profileStart = cv::Point (std::round((XY_start[1] - printROI[1]) / PIX2MM), std::round((XY_start[0] - printROI[0]) / PIX2MM));
	profileEnd = cv::Point (std::round((XY_end[1] - printROI[1]) / PIX2MM), std::round((XY_end[0] - printROI[0]) / PIX2MM));
	profileROIRange = cv::Range(startIdx, endIdx);
	return;
}