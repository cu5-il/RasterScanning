#include <iostream>
#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "draw.h"

// If cvPlot is installed, use the functions
#  if __has_include(<CvPlot/cvplot.h>)
#    include <CvPlot/cvplot.h>

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

cv::Mat plotScan(cv::Mat scanROI, bool showImage = false) {
	cv::Mat image1, image2, image3;

	// HACK: calculating the derivative separately for plotting function
	cv::Mat dx, dx2, scanROIblur;
	int aperture_size = 7;
	int sigma = 11;
	int sz = 19;
	cv::GaussianBlur(scanROI, scanROIblur, cv::Size(sz, sz), (double)sigma / 10);
	cv::Sobel(scanROIblur, dx, -1, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
	cv::Sobel(scanROIblur, dx2, -1, 2, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);

	//Plotting
	
	auto axes_dx = CvPlot::makePlotAxes();
	axes_dx.create<CvPlot::Series>(dx, "-b");

	auto axes_dx2 = CvPlot::makePlotAxes();
	axes_dx2.create<CvPlot::Series>(dx2, "-r");


	auto axes_prfl = CvPlot::makePlotAxes();
	axes_prfl.create<CvPlot::Series>(scanROI, "-k");
	axes_prfl.create<CvPlot::Series>(scanROIblur, "-b");
	//axes_prfl.create<CvPlot::Series>(dx2, "-r");

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

		image3 = axes_dx2.render();
		cv::namedWindow("2nd Derivative", cv::WINDOW_NORMAL);
		cv::setMouseCallback("2nd Derivative", mouse_callback);
		cv::imshow("2nd Derivative", image3);
		cv::waitKey(0);
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
#  endif

