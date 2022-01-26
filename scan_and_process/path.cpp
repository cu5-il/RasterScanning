#include <iostream>
#include <vector>
#include <iterator> 
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"
#include "raster.h"


/**
 * @brief Breaks up the raster pattern into segments. Each vertical rod in the raster is a segment; corners are neglected.
 * @param[in] rasterCoords Vector of (x,y) points of the raster pattern
 * @param[in] ROIwidth of the ROI
 * @param[out] ROIs Vector of regions that are around each rod. Used to determine which edges belong to which segment
 * @param[out] centerlines Vector of point pairs defining the centerline of the region
 * @param[out] scanDonePts Point in the raster pattern when the region has been finished scanning. Set to midpoint of the following rod except for the last region, which is just the offset of the last point of the raster
*/
void segmentPath(Raster raster, double ROIwidth, std::vector<Segment>& seg, cv::Point2d initPos) {
	
	std::vector<cv::Point> rasterCoords = raster.px();
	std::vector<cv::Rect> ROIs;
	std::vector<std::vector<cv::Point>> centerlines;
	std::vector<cv::Point2d> scanDonePts;
	int pixWidth = raster.rodWidth();
	int direction = 1;
	// Rods
	for (auto it = rasterCoords.begin(); it != rasterCoords.end(); std::advance(it, 2)) {
		ROIs.push_back(cv::Rect(*it - cv::Point(0, pixWidth / 2), *std::next(it, 1) + cv::Point(0, pixWidth / 2)));
		centerlines.push_back(std::vector<cv::Point> { *it, * std::next(it, 1)});
	}
	// defining the point when the region has been completely scanned as the midpoint of the next centerline
	for (auto it = std::next(centerlines.begin()); it != centerlines.end(); ++it) {
		scanDonePts.push_back(PIX2MM((cv::Point2d((*it).front() + (*it).back()) / 2 - cv::Point2d(rasterCoords.front()))) + initPos);
	}
	// TODO: change scanning termination point to last point in raster + scanner offset
	// Make the point to end scanning for the final region the last point in the raster
	scanDonePts.push_back(PIX2MM((cv::Point2d(rasterCoords.back()) - cv::Point2d(rasterCoords.front()))) + initPos);

	// Placing all of the values in the Segment class
	if ((ROIs.size() == centerlines.size()) && (ROIs.size() == scanDonePts.size())) {
		for (int i = 0; i < ROIs.size(); i++) {
			seg.push_back(Segment(ROIs[i], centerlines[i], scanDonePts[i], direction));
			direction *= -1; // flip direction
		}
	}

	// Corners
	// TODO: add corner regions
}

void interpolatePath(Raster raster, double waypointSpacing, std::vector<std::vector<cv::Point2d>>& path_mm, std::vector<std::vector<cv::Point>>& path_px) {
	cv::Point2d diff, delta;
	double dX, dY, L;
	std::vector<cv::Point2d> tmp_mm;
	std::vector<cv::Point> tmp_px;

	// Interpolate the waypoints between corners
	for (auto it = raster.mm().begin(); it != std::prev(raster.mm().end(), 1); ++it) {

		diff = *std::next(it, 1) - *it;
		L = cv::norm(diff) / waypointSpacing;
		delta = diff / L;
		dX = diff.x / L;
		dY = diff.y / L;
		for (int i = 0; cv::norm(i*delta) <= L; i++) {
			tmp_mm.push_back(cv::Point2d(*it) + i * delta);
			tmp_px.push_back(cv::Point( ((tmp_mm.back() - raster.origin())/RESOLUTION )  ));
		}
		path_mm.push_back(tmp_mm);
		path_px.push_back(tmp_px);
		tmp_mm.clear();
		tmp_px.clear();
	}
}

void readPath(std::string filename, double& rodLength, double& rodSpacing, std::deque<std::vector<double>>& path)
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
				path.push_back(vec);
				break;
				// Read the header data
			case 0:
				rodLength = std::stod(single_line);
				headerCnt++;
				break;
			case 1:
				rodSpacing = std::stod(single_line);
				headerCnt++;
				break;
			}
		}
	}
	else {
		std::cout << "Unable to open data file: " << filename << std::endl;;
		system("pause");
		return;
	}

	return;
}