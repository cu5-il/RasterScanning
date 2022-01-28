#include <iostream>
#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


/**
 * @brief callback function for when mouse button is clicked on an image
*/
void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == cv::EVENT_LBUTTONDOWN) {
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
}

/**
 * @brief Overlays the most recent scan on the raster pattern
 * @param[in] Mat conting the raster pattern
 * @param[in[ scanROI Profile from the scanner that is within the print ROI
 * @param[in] scanStart Start coordinates of the scan
 * @param[in] scanEnd End coordinates of the scan
 * @param[in] showImage Flag indicating whether to show the overlay in a new window or not
 * @return Mat containing the scan overlaid on the raster pattern 
*/
cv::Mat showOverlay(cv::Mat raster, cv::Mat scanROI, const Coords fbk, cv::Point scanStart, cv::Point scanEnd, bool showImage = false) {
	cv::Mat scanTall, scanGray, image, rasterInv;
	int height = 2;
	
	//if ((scanStart.x >= 0) && (scanStart.x <= raster.cols) && (scanEnd.x >= 0) && (scanEnd.x <= raster.cols)&&
	//	(scanStart.y >= 0) && (scanStart.y <= raster.rows) && (scanEnd.y >= 0) && (scanEnd.y <= raster.rows))
	//{
		//Converting scanROI into a color image
		cv::normalize(scanROI, scanGray, 0, 255, cv::NORM_MINMAX, CV_8U);
		cv::cvtColor(scanGray, scanGray, cv::COLOR_GRAY2BGR);
		// Stretching the scan
		cv::resize(scanGray, scanTall, cv::Size(scanGray.cols, height), cv::INTER_LINEAR);
		// Copy the raster to the image
		cv::Mat(raster.size(), CV_8UC3, cv::Scalar({ 0, 255, 0, 0 })).copyTo(image, raster);
		// Copy the scan to the raster
		bitwise_not(raster, rasterInv);
		//scanTall.copyTo(image(cv::Rect(scanStart, scanTall.size())), rasterInv(cv::Rect(scanStart, scanTall.size())));
		// Draw a line where scan was taken
		cv::line(image, scanStart, scanEnd, cv::Scalar(0, 0, 255), 2);

		// Draw the position of the nozzle
		cv::Point nozzle =cv::Point(MM2PIX(fbk.x-45), MM2PIX(fbk.y-15));
		cv::circle(image, nozzle, 10, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);

		if (showImage) {
			// Display the image
			cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
			cv::setMouseCallback("Overlay", mouse_callback);
			cv::imshow("Overlay", image);
			cv::waitKey(1);
		}
	//}
		return image;
}

/**
 * @brief Overlays the detected edges and search windows on the scan profile
 * @param[in] scanROI Profile from the scanner that is within the print ROI
 * @param[in] locEdges Mat containing the location of all the found edges in the global coordinate system
 * @param[in] locWin Mat containing the location of the search windows
 * @param[in] showImage Flag indicating whether to show the overlay in a new window or not
 * @return Mat containing the scan overlaid on the raster pattern
*/
cv::Mat showScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin, bool showImage = false) {
	cv::Mat image;
	int height = 40;

	//Converting scanROI into a color image
	cv::normalize(scanROI, image, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
	// Copy the edges to the image
	cv::Mat(image.size(), CV_8UC3, cv::Scalar({ 255, 0, 255, 0 })).copyTo(image, locEdges);
	// Copy the search windows to the image
	cv::Mat(image.size(), CV_8UC3, cv::Scalar({ 255, 0, 0, 0 })).copyTo(image, locWin);
	
	if (showImage) {
		// Display the image
		cv::namedWindow("Scan", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Scan", mouse_callback);
		cv::imshow("Scan", image);
		cv::waitKey(1);
	}
	

	return image;
}

/**
 * @brief Overlay the detected edges on the raster
 * @param[in] raster Mat containing the raster pattern
 * @param[in] gblEdges Mat containing the location of all the found edges in the global coordinate system
 * @param[in] color 
 * @param[in] pointSz Size of the circles placed at the location of the detected edges. A negative value will show the edges as pixels
 * @param[in] showImage Flag indicating whether to show the overlay in a new window or not
 * @return Mat containing the detected edges overlaid on the raster pattern
*/
cv::Mat showRaster(cv::Mat raster, cv::Mat edgeBoundary, cv::Mat gblEdges, const cv::Scalar& color, const int pointSz = 1, bool showImage = false) {
	cv::Mat image = cv::Mat::zeros(raster.size(), CV_8UC3);

	// Copy the raster to the image
	cv::cvtColor(raster, image, cv::COLOR_GRAY2BGR);
	// Copy the global edges to the  image
	if (pointSz > 0) {
		// show the edges as filled circles
		std::vector<cv::Point> gblEdgePts;
		cv::findNonZero(gblEdges, gblEdgePts);
		for (auto it = gblEdgePts.begin(); it != gblEdgePts.end(); ++it) {
			cv::circle(image, *it, pointSz, color, -1, cv::LINE_AA);
		}
	}
	else {
		// show the edges as pixels
		cv::Mat(raster.size(), CV_8UC3, color).copyTo(image, gblEdges);
	}
	// draw the boundary on the image
	std::vector<std::vector<cv::Point> > contour;
	cv::findContours(edgeBoundary, contour, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	cv::drawContours(image, contour, -1, cv::Scalar(255, 0, 0));

	if (showImage) {
		// Display the image
		cv::namedWindow("Raster w/ Edges", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Raster w/ Edges", mouse_callback);
		cv::imshow("Raster w/ Edges", image);
		cv::waitKey(1);
	}

	return image;
}

/**
 * @brief Creates a single image combining the outputs of the showOverlay, showScan, and showRaster functions
 * @param[in] raster Mat containing the raster pattern
 * @param[in] scanROI Profile from the scanner that is within the print ROI
 * @param[in] scanStart Start coordinates of the scan
 * @param[in] scanEnd End coordinates of the scan
 * @param[in] locEdges Mat containing the location of all the found edges in the global coordinate system
 * @param[in] locWin Mat containing the location of the search windows
 * @param[in] gblEdges Mat containing the location of all the found edges in the global coordinate system
 * @param[in] showImage Flag indicating whether to show the overlay in a new window or not
 * @return Mat contining the image
*/
cv::Mat showAll(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd, cv::Mat locEdges, cv::Mat locWin, cv::Mat gblEdges, bool showImage = false) {
	cv::Mat image;

	// making the images
	//cv::Mat overlay_img = showOverlay(raster, scanROI, scanStart, scanEnd, false);
	cv::Mat scan_img = showScan(scanROI, locEdges, locWin, false);
	//cv::Mat raster_img = showRaster(raster, gblEdges, false);

	// Stretching the local scan image
	cv::resize(scan_img, scan_img, cv::Size(scan_img.cols, 100), cv::INTER_LINEAR);

	//cv::Mat image(cv::Size(overlay_img.cols + raster_img.cols, scan_img.rows + raster_img.rows), CV_8UC3);

	// Copy the individual images to the combined image
	//raster_img.copyTo(image(cv::Rect(cv::Point(0, 0), raster_img.size())));
	//scan_img.copyTo(image(cv::Rect(cv::Point(0, raster_img.rows), scan_img.size())));
	//overlay_img.copyTo(image(cv::Rect(cv::Point(raster_img.cols, 0), overlay_img.size())));

	if (showImage) {
		// Display the image
		cv::namedWindow("Combined Image", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Combined Image", mouse_callback);
		cv::imshow("Combined Image", image);
		cv::waitKey(1);
	}

	return image;
}

/**
 * @brief Adds a scale bar to an image
 * @param image Mat containing the image
 * @param offset Offset from the bottom left corner of the image where the scale will be placed
*/
void addScale(cv::Mat& image, cv::Point offset = cv::Point(25,25) ) {
	cv::Point location(offset.x, image.rows - offset.y);
	cv::putText(image, "1mm", location, cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(255, 255, 255), 1, cv::LINE_8);
	cv::rectangle(image, cv::Rect(location.x + 6, location.y + 5, MM2PIX(1), 5), cv::Scalar(255, 255, 255), -1);
}

/**
 * @brief 
 * @param src 
 * @param dst 
 * @param seg 
*/
void showErrors(cv::InputArray src, cv::OutputArray dst, std::vector<Segment>& seg) {
	std::vector<cv::Point> allEdgePts, actCenterline, lEdgeErr, rEdgeErr;
	cv::Mat tempLines= cv::Mat::zeros(src.size(), CV_8UC3);
	cv::Mat tempFill = cv::Mat::zeros(src.size(), CV_8UC3);
	cv::Mat mask = cv::Mat(src.size(), CV_8UC3);

	// getting the values from each segment
	for (auto it = seg.begin(); it != seg.end(); ++it) {
		// check if there are edge points
		if (!(*it).lEdgePts().empty() && !(*it).rEdgePts().empty() ) {
			// Draw material edges
			cv::polylines(tempLines, (*it).lEdgePts(), false, cv::Scalar(255, 255, 0), 1);
			cv::polylines(tempLines, (*it).rEdgePts(), false, cv::Scalar(255, 255, 0), 1);
			// Fill the region between the edges
			allEdgePts.reserve((*it).lEdgePts().size() + (*it).rEdgePts().size()); // preallocate memory
			allEdgePts.insert(allEdgePts.end(), (*it).lEdgePts().begin(), (*it).lEdgePts().end());
			allEdgePts.insert(allEdgePts.end(), (*it).rEdgePts().rbegin(), (*it).rEdgePts().rend());
			cv::fillPoly(tempFill, allEdgePts, cv::Scalar(255, 255, 0));
			allEdgePts.clear();
		}
		// check if the errors have been calculated
		if (!(*it).errCL().empty() && !(*it).errWD().empty() ) {
			for (int i = 0; i < (*it).errCL().size(); i++) {
				actCenterline.push_back((*it).waypoints()[i] + cv::Point(0, (int)round((*it).errCL()[i])));
				lEdgeErr.push_back(actCenterline.back() - cv::Point(0, (int)round((*it).errWD()[i] / 2)));
				rEdgeErr.push_back(actCenterline.back() + cv::Point(0, (int)round((*it).errWD()[i] / 2)));
			}
			// Draw the errors
			cv::polylines(tempLines, lEdgeErr, false, cv::Scalar(0, 255, 255), 1);
			cv::polylines(tempLines, rEdgeErr, false, cv::Scalar(0, 255, 255), 1);
			cv::polylines(tempLines, actCenterline, false, cv::Scalar(0, 0, 255), 1);
			lEdgeErr.clear();
			rEdgeErr.clear();
			actCenterline.clear();
		}
	}
	// copy the source to the destination
	src.copyTo(dst);
	if (dst.channels() < 3) {
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
	}
	// overlay the fill on the destination
	cv::addWeighted(tempFill, 0.5, dst, 1, 0, dst);
	// add the lines to the destination
	cv::cvtColor(tempLines, mask, cv::COLOR_BGR2GRAY);
	cv::threshold(mask, mask, 1, 255, cv::THRESH_BINARY);
	tempLines.copyTo(dst, mask);
}