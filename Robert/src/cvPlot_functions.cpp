#include "cvPlot_functions.h"
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

cv::Mat plotScan(cv::Mat scanROI, cv::Mat locEdges, cv::Mat locWin, bool showImage ) {
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

cv::Mat plotScan(cv::Mat scanROI, std::vector<cv::Mat> mats) {
	cv::Mat image1, image2, image3;

	// HACK: calculating the derivative separately for plotting function
	cv::Mat dx, dx2, scanROIblur;
	int aperture_size = 7;
	int sigma = 61;
	int sz = 9;
	cv::Mat dxBlur;
	cv::GaussianBlur(scanROI, scanROIblur, cv::Size(sz, sz), (double)sigma / 10);
	cv::Sobel(scanROIblur, dx, -1, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
	cv::Sobel(scanROIblur, dx2, -1, 2, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);

	double minV, maxV;
	cv::Mat median;
	cv::Scalar mean, stddev;
	int op ;
	int mSz = 5;
	int iter = 2;
	cv::Mat k;
	cv::Mat morph, aa, morph2;
	
	cv::Mat normed, otsu;
	cv::normalize(scanROIblur, normed, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::threshold(normed, median, 2, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU);

	cv::meanStdDev(scanROI, mean, stddev);
	// find the baseline average
	cv::minMaxIdx(scanROIblur, &minV, &maxV, NULL, NULL);
	//cv::threshold(scanROIblur, median, (maxV + minV) / 2, 255, cv::THRESH_BINARY_INV);
	median.convertTo(median, CV_8U);
	k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::morphologyEx(median, median, cv::MORPH_CLOSE, k, cv::Point(-1, -1), 1);
	cv::meanStdDev(scanROI, mean, stddev, median);


	// closing method
	cv::Mat baseline;// = (mean.val[0] * cv::Mat::ones(scanROIblur.size(), scanROIblur.type()));
	//scanROIblur.copyTo(baseline);
	cv::threshold(scanROIblur, baseline, mean.val[0] , 255, cv::THRESH_TRUNC);
	op =  cv::MORPH_OPEN;
	op = cv::MORPH_ERODE;
	//iter = 30;
	//mSz = 9;
	//k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(mSz, mSz));
	cv::morphologyEx(baseline, baseline, op, k, cv::Point(-1, -1), iter);
	int blursz = scanROI.cols / 8;
	cv::blur(baseline, baseline, cv::Size(blursz, blursz));


	iter = 2;
	scanROIblur.copyTo(morph2);
	k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	//cv::morphologyEx(median, median, cv::MORPH_ERODE, k, cv::Point(-1, -1), 1);
	baseline.copyTo(morph2, median);
	k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	op = cv::MORPH_CLOSE;
	cv::morphologyEx(morph2, morph2, op, k, cv::Point(-1, -1), iter);
	//cv::threshold(morph2, morph2, mean.val[0]+stddev.val[0], mean.val[0], cv::THRESH_BINARY);
	sigma = 19;
	cv::GaussianBlur(morph2, morph2, cv::Size(sz, sz), (double)sigma / 10);

	cv::Mat edgeMask, edges;
	cv::compare(morph2, baseline+0.001, edgeMask, cv::CMP_GT);

	cv::Sobel(edgeMask, dx2, -1, 2, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
	baseline.copyTo(edges, edgeMask);

	
	auto axes_1 = CvPlot::makePlotAxes();
	axes_1.create<CvPlot::Series>(scanROI, "-k");
	axes_1.create<CvPlot::Series>(scanROIblur, "-b");
	//axes_1.create<CvPlot::Series>(morph, "-r");
	axes_1.create<CvPlot::Series>(morph2, "-m");
	axes_1.create<CvPlot::Series>(baseline, "-c");
	//axes_1.create<CvPlot::Series>(edges, "-r");
	aa = axes_1.render();

	auto axes_2 = CvPlot::makePlotAxes();
	axes_2.create<CvPlot::Series>(scanROI, "-k");
	axes_2.create<CvPlot::Series>(mats[0], "-b");
	axes_2.create<CvPlot::Series>(mats[1], "-m");
	axes_2.create<CvPlot::Series>(mats[2], "-c");
	cv::Mat ab = axes_2.render();


	return aa;
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

