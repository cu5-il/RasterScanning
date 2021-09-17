#include <iostream>

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == cv::EVENT_LBUTTONDOWN) {
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
}

cv::Mat showOverlay(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd) {
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
		scanTall.copyTo(image(cv::Rect(scanStart, scanTall.size())), rasterInv(cv::Rect(scanStart, scanTall.size())));
		// Draw a line where scan was taken
		cv::line(image, scanStart, scanEnd, cv::Scalar(0, 0, 255), 2);

		// Display the image
		cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Overlay", mouse_callback);
		cv::imshow("Overlay", image);
		cv::waitKey(1);
	//}
		return image;
}

cv::Mat showScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin) {
	cv::Mat image;
	int height = 40;

	//Converting scanROI into a color image
	cv::normalize(scanROI, image, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
	// Copy the edges to the image
	cv::Mat(image.size(), CV_8UC3, cv::Scalar({ 255, 0, 255, 0 })).copyTo(image, locEdges);
	// Copy the search windows to the image
	cv::Mat(image.size(), CV_8UC3, cv::Scalar({ 255, 0, 0, 0 })).copyTo(image, locWin);
	
	// Display the image
	cv::namedWindow("Scan", cv::WINDOW_NORMAL);
	cv::setMouseCallback("Scan", mouse_callback);
	cv::imshow("Scan", image);
	cv::waitKey(1);

	// Stretching the image
	cv::resize(image, image, cv::Size(image.cols, 100), cv::INTER_LINEAR);

	return image;
}

cv::Mat showRaster(cv::Mat& raster, cv::Mat gblEdges) {
	cv::Mat image;

	// Copy the raster to the image
	raster.copyTo(image);
	// Copy the global edges to the  image
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar({ 0, 0, 255, 0 })).copyTo(image, gblEdges);

	// Display the image
	cv::namedWindow("Raster w/ Edges", cv::WINDOW_NORMAL);
	cv::setMouseCallback("Raster w/ Edges", mouse_callback);
	cv::imshow("Raster w/ Edges", image);
	cv::waitKey(1);

	return image;
}