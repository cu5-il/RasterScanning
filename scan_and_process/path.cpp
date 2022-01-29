#include <iostream>
#include <vector>
#include <iterator> 
#include <algorithm>
#include <opencv2/core.hpp>

#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"
#include "raster.h"

void interpPathPoints(std::vector<cv::Point2d> inPts, double wayptSpc, std::vector<cv::Point2d>& outPts) {
	cv::Point2d diff, delta;
	double L;

	// Interpolate the points
	for (auto it = inPts.begin(); it != std::prev(inPts.end(), 1); ++it) {
		diff = *std::next(it, 1) - *it;
		L = cv::norm(diff) / wayptSpc;
		delta = diff / L;
		for (int i = 0; cv::norm(i*delta) < L; i++) {
			outPts.push_back(cv::Point2d(*it) + i * delta);
		}
	}
}

void interpPathPoints(std::vector<cv::Point2i> inPts, double wayptSpc, std::vector<cv::Point2i>& outPts) {
	cv::Point2d diff, delta;
	double L;

	// Interpolate the points
	for (auto it = inPts.begin(); it != std::prev(inPts.end(), 1); ++it) {
		diff = *std::next(it, 1) - *it;
		L = cv::norm(diff);
		delta = (diff / L) * (double) MM2PIX(wayptSpc);
		for (int i = 0; cv::norm(i * delta) < L; i++) {
			outPts.push_back(*it + cv::Point2i(i * delta));
		}
	}
}

bool makePath(Raster raster, double wayptSpc, std::deque<double>& theta, cv::Point3d initPos, double initVel, double initExt, std::vector<Segment>& seg, std::vector<std::vector<Path>>& path) {

	std::vector<cv::Point> rasterCoords = raster.px();
	cv::Rect roi;
	std::vector<cv::Point2i> wp_px;
	std::vector<cv::Point2d> wp_mm;
	cv::Point2d scanDonePt;
	int pixWidth = raster.rodWidth();
	int direction = 1;
	cv::Point2d origin = raster.origin();
	std::vector<Path> tmp;

	double z = initPos.z;
	double T = 90;
	double e = initExt;
	double f = initVel;

	// Rods
	for (auto it = raster.px().begin(); it != std::prev(raster.px().end()); ++it) {
		// Generate the waypoints
		interpPathPoints(std::vector<cv::Point2i> {*it, * std::next(it)}, wayptSpc, wp_px);
		// add the final point of the pattern
		if (std::next(it) == std::prev(raster.px().end())) {
			wp_px.push_back(*std::next(it));
		}
		wp_mm.resize(wp_px.size());
		std::transform(wp_px.begin(), wp_px.end(), wp_mm.begin(), [&origin](cv::Point& pt) {return (PIX2MM(cv::Point2d(pt)) + origin); });

		for (auto it2 = wp_mm.begin(); it2 != wp_mm.end(); ++it2) {
			if (!theta.empty()) {
				T = theta.front();
				tmp.push_back(Path(*it2, z, T, f, e));
				theta.pop_front();
			}
			else {
				std::cout << "Not enough theta points" << std::endl;
				system("pause");
				return false;
			}
		}
		path.push_back(tmp);

		// Determine the direction
		if (((*std::next(it)).x - (*it).x) > 0) { direction = 0; }		// positive x direction
		else if (((*std::next(it)).x - (*it).x) < 0) { direction = 2; } // negative x direction
		else if (((*std::next(it)).y - (*it).y) > 0) { direction = 1; } // positive y direction
		else if (((*std::next(it)).y - (*it).y) < 0) { direction = 3; } // negative y direction

		// Defining the regions and the point when the region has been completely scanned
		switch (direction % 2) {
		case 0: 
			roi = cv::Rect(*it - cv::Point(0, raster.rodWidth() / 2), *std::next(it, 1) + cv::Point(0, raster.rodWidth() / 2));
			// defining the point when the region has been completely scanned as the midpoint of the next horizontal line
			scanDonePt = (wp_mm.front() + wp_mm.back()) / 2 + cv::Point2d(wayptSpc/2, raster.spacing());
			// if it is the final segment
			if (std::next(it) == std::prev(raster.px().end())) {
				scanDonePt = wp_mm.back() ;
			}
			break;
		case 1: // vertical lines
			roi = cv::Rect(0,0,1,1);
			//roi = cv::Rect(*it - cv::Point(raster.rodWidth() / 2, 0), *std::next(it, 1) + cv::Point(raster.rodWidth() / 2, 0));
			// defining the point when the region has been completely scanned as 3/4 of the length of the next horizontal line
			scanDonePt = wp_mm.front() + cv::Point2d(raster.length() * 0.75, raster.spacing() );
			if (!raster.roi().contains(scanDonePt)) {
				scanDonePt -= cv::Point2d(raster.length() * 1.5, 0);
			}
			break;
		}
		//TODO: change segment class to include waypoints in mm
		seg.push_back(Segment(roi, wp_px, scanDonePt, direction));
		wp_px.clear();
		wp_mm.clear();
		tmp.clear();
	}

	if (!theta.empty()) {
		std::cout << "Not all theta points assigned " << std::endl;
		system("pause");
		return false;
	}
	return true;
}

void readPath(std::string filename, double& rodLen, double& rodSpc, double& wayptSpc, std::deque<std::vector<double>>& path, std::deque<double>& theta)
{
	std::ifstream inFile(filename.c_str());
	std::string single_line;
	double value;
	int headerCnt = 0;

	if (inFile.is_open()) {
		while (std::getline(inFile, single_line)) {
			std::vector<double> vec;
			std::stringstream temp(single_line);
			std::string single_value;

			switch (headerCnt)
			{
			default: // Read the path data
				while (std::getline(temp, single_value, ',')) {
					value = std::stod(single_value);
					vec.push_back(value);
				}
				theta.push_back(value);
				path.push_back(vec);
				break;
				// Read the header data
			case 0:
				rodLen = std::stod(single_line);
				headerCnt++;
				break;
			case 1:
				rodSpc = std::stod(single_line);
				headerCnt++;
				break;
			case 2:
				wayptSpc = std::stod(single_line);
				headerCnt++;
				break;
			}
		}
	}
	else {
		std::cout << "Unable to open data file: " << filename << std::endl;
		system("pause");
		return;
	}

	return;
}