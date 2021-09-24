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

cv::Mat showOverlay(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd, bool showImage) {
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

cv::Mat showScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin, bool showImage) {
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

cv::Mat showRaster(cv::Mat raster, cv::Mat gblEdges, bool showImage) {
	cv::Mat image;

	// Copy the raster to the image
	raster.copyTo(image);
	// Copy the global edges to the  image
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar({ 255, 0, 255, 0 })).copyTo(image, gblEdges);

	if (showImage) {
		// Display the image
		cv::namedWindow("Raster w/ Edges", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Raster w/ Edges", mouse_callback);
		cv::imshow("Raster w/ Edges", image);
		cv::waitKey(1);
	}

	return image;
}

cv::Mat showAll(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd, cv::Mat locEdges, cv::Mat locWin, cv::Mat gblEdges, bool showImage) {
	
	// making the images
	cv::Mat overlay_img = showOverlay(raster, scanROI, scanStart, scanEnd, false);
	cv::Mat scan_img = showScan(scanROI, locEdges, locWin, false);
	cv::Mat raster_img = showRaster(raster, gblEdges, false);

	// Stretching the local scan image
	cv::resize(scan_img, scan_img, cv::Size(scan_img.cols, 100), cv::INTER_LINEAR);

	cv::Mat image(cv::Size(overlay_img.cols + raster_img.cols, scan_img.rows + raster_img.rows), CV_8UC3);

	// Copy the individual images to the combined image
	raster_img.copyTo(image(cv::Rect(cv::Point(0, 0), raster_img.size())));
	scan_img.copyTo(image(cv::Rect(cv::Point(0, raster_img.rows), scan_img.size())));
	overlay_img.copyTo(image(cv::Rect(cv::Point(raster_img.cols, 0), overlay_img.size())));

	if (showImage) {
		// Display the image
		cv::namedWindow("Combined Image", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Combined Image", mouse_callback);
		cv::imshow("Combined Image", image);
		cv::waitKey(1);
	}

	return image;
}