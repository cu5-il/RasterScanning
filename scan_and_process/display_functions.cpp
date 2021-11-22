#include <iostream>

#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <CvPlot/cvplot.h>

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
cv::Mat showOverlay(cv::Mat raster, cv::Mat scanROI, cv::Point scanStart, cv::Point scanEnd, bool showImage = false) {
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
cv::Mat showRaster(cv::Mat raster, cv::Mat gblEdges, const cv::Scalar& color, const int pointSz = 1, bool showImage = false) {
	cv::Mat image;

	// Copy the raster to the image
	cv::cvtColor(raster, image, cv::COLOR_GRAY2BGR);
	// Copy the global edges to the  image
	if (pointSz > 0) {
		// show the edges as filled circles
		std::vector<cv::Point> gblEdgePts;
		cv::findNonZero(gblEdges, gblEdgePts);
		for (auto it = gblEdgePts.begin(); it != gblEdgePts.end(); ++it) {
			cv::circle(image, *it, 1, color, -1, cv::LINE_AA);
		}
	}
	else {
		// show the edges as pixels
		cv::Mat(raster.size(), CV_8UC3, color).copyTo(image, gblEdges);
	}
	

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

// Functions using CV-Plot

cv::Mat plotScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin, bool showImage = false) {
	cv::Mat image1, image2;

	// HACK: calculating the derivative separately for plotting function
	cv::Mat dx, scanROIblur;
	int aperture_size = 7;
	int sigma = 61;
	int sz = 19;
	cv::GaussianBlur(scanROI, scanROIblur, cv::Size(sz, sz), (double)sigma / 10);
	cv::Sobel(scanROIblur, dx, -1, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);

	//Plotting
	cv::Mat pltEdges(scanROI.size(), CV_32F, cv::Scalar(NAN));
	cv::Mat pltWins(scanROI.size(), CV_32F, cv::Scalar(NAN));
	// HACK: need to repeat setting the matrix to Nan because they get change to 0 after first copyTo
	dx.copyTo(pltEdges, locEdges);
	dx.copyTo(pltWins, locWin);
	pltEdges = cv::Scalar(NAN);
	pltWins = cv::Scalar(NAN);
	dx.copyTo(pltEdges, locEdges);
	dx.copyTo(pltWins, locWin);

	auto axes_dx = CvPlot::makePlotAxes();
	axes_dx.create<CvPlot::Series>(dx, "-k");
	axes_dx.create<CvPlot::Series>(pltWins, "bo");
	axes_dx.create<CvPlot::Series>(pltEdges, "ro");

	pltEdges = cv::Scalar(NAN);
	pltWins = cv::Scalar(NAN);
	scanROI.copyTo(pltEdges, locEdges);
	scanROI.copyTo(pltWins, locWin);
	auto axes_prfl = CvPlot::makePlotAxes();
	axes_prfl.create<CvPlot::Series>(scanROI, "-k");
	axes_prfl.create<CvPlot::Series>(pltWins, "bo");
	axes_prfl.create<CvPlot::Series>(pltEdges, "ro");

	if (showImage) {
		// Display the image
		image1 = axes_prfl.render();
		cv::namedWindow("Scan", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Scan", mouse_callback);
		cv::imshow("Scan", image1);

		image2 = axes_dx.render();
		cv::namedWindow("Derivative", cv::WINDOW_NORMAL);
		cv::setMouseCallback("Derivative", mouse_callback);
		cv::imshow("Derivative", image2);
		cv::waitKey(1);
	}

	return image1;
}


void plotEdges(const std::vector<cv::Point>& unfiltered, const std::vector<cv::Point>& filtered) {
	std::vector<double> raw, smooth;
	for (auto it = unfiltered.begin(); it != unfiltered.end(); ++it) {
		raw.push_back((*it).x);
	}
	for (auto it = filtered.begin(); it != filtered.end(); ++it) {
		smooth.push_back((*it).x);
	}

	cv::namedWindow("edge", cv::WINDOW_NORMAL);
	auto axes = CvPlot::makePlotAxes();
	axes.create<CvPlot::Series>(raw, "-k");
	axes.create<CvPlot::Series>(smooth, "-r");
	CvPlot::show("edge", axes);
}
