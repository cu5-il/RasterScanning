#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


void processData(double collectedData[][NUM_DATA_SAMPLES], Coords* fbk, cv::Mat& profile) {
	int fbIdx[2];
	int profileStartIdx;
	cv::Mat profileVoltage_8U, profileEdges, profileEdgesIdx;

	cv::Mat dataMat(NUM_PROFILE_PTS, NUM_DATA_SAMPLES, CV_64F, collectedData); // copying the collected data into a matrix

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