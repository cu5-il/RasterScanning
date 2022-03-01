#include "gaussianSmooth.h"
#include <iostream>
#include <vector>
#include <valarray>
#include <iterator> 
#include <cmath>
#include <opencv2/core.hpp>

#include "myGlobals.h"
#include "constants.h"

/**
 * @brief Applies a discrete Gaussian filter to smooth a vector of points. Smooths only in the x direction.
 * @param[in] unfiltPts Vector of unfiltered points
 * @param[out] filtPts Vector of points after applying the filter
 * @param[in] kSize Number of points to before (or after) the target point to smooth ( kSize = 3 corresponds to a kernel size of 7 )
 * @param[in] sig Standard deviation of the Gaussian kernel
*/
void gaussianSmoothX(const std::vector<cv::Point>& unfiltPts, std::vector<cv::Point>& filtPts, int kSize, double sig) {
	// apply gaussian smoothing to the x points of a polyline
	double sumK, filteredVal;
	filtPts.clear();
	filtPts.reserve(unfiltPts.size());
	// make the discrete Gaussian kernel
	std::valarray<double> K(2 * kSize + 1);
	for (int i = 0; i < 2 * kSize + 1; i++) {
		K[i] = exp(-pow((i - kSize), 2) / (2 * pow(sig, 2)));
	}
	K = K / K.sum();

	for (auto it = unfiltPts.begin(); it != unfiltPts.end(); ++it) {
		sumK = 0;
		// contribution of sample point
		filteredVal = (*it).y * K[kSize];
		sumK += K[kSize];
		// forward loop for points ahead of sample point
		for (auto itf = std::next(it, 1); itf != unfiltPts.end() && std::distance(it, itf) <= kSize; ++itf) {
			sumK += K[kSize + std::distance(it, itf) ];
			filteredVal += (*itf).y * K[kSize + std::distance(it, itf)];
		}
		// reverse loop for points behind of sample point
		for (auto itr = it; itr != unfiltPts.begin() && std::distance(itr, it) < kSize; --itr) {
			sumK += K[kSize - std::distance(std::prev(itr,1), it) ];
			filteredVal += (*std::prev(itr, 1)).y * K[kSize - std::distance(std::prev(itr, 1), it)];
		}
		filtPts.push_back(cv::Point((*it).x, (int)(filteredVal / sumK) ));
	}
}

/**
 * @brief Applies a discrete Gaussian filter to smooth a vector of points. Smooths only in the Y direction.
 * @param[in] unfiltPts Vector of unfiltered points
 * @param[out] filtPts Vector of points after applying the filter
 * @param[in] kSize Number of points to before (or after) the target point to smooth ( kSize = 3 corresponds to a kernel size of 7 )
 * @param[in] sig Standard deviation of the Gaussian kernel
*/
void gaussianSmoothY(const std::vector<cv::Point>& unfiltPts, std::vector<cv::Point>& filtPts, int kSize, double sig) {
	// apply gaussian smoothing to the y points of a polyline
	double sumK, filteredVal;
	filtPts.clear();
	filtPts.reserve(unfiltPts.size());
	// make the discrete Gaussian kernel
	std::valarray<double> K(2 * kSize + 1);
	for (int i = 0; i < 2 * kSize + 1; i++) {
		K[i] = exp(-pow((i - kSize), 2) / (2 * pow(sig, 2)));
	}
	K = K / K.sum();

	for (auto it = unfiltPts.begin(); it != unfiltPts.end(); ++it) {
		sumK = 0;
		// contribution of sample point
		filteredVal = (*it).x * K[kSize];
		sumK += K[kSize];
		// forward loop for points ahead of sample point
		for (auto itf = std::next(it, 1); itf != unfiltPts.end() && std::distance(it, itf) <= kSize; ++itf) {
			sumK += K[kSize + std::distance(it, itf)];
			filteredVal += (*itf).x * K[kSize + std::distance(it, itf)];
		}
		// reverse loop for points behind of sample point
		for (auto itr = it; itr != unfiltPts.begin() && std::distance(itr, it) < kSize; --itr) {
			sumK += K[kSize - std::distance(std::prev(itr, 1), it)];
			filteredVal += (*std::prev(itr, 1)).x * K[kSize - std::distance(std::prev(itr, 1), it)];
		}
		filtPts.push_back(cv::Point((int)(filteredVal / sumK), (*it).y));
	}
}