#include <iostream>
#include <vector>
#include <valarray>
#include <iterator> 
#include <cmath>
#include <opencv2/core.hpp>

#include "myGlobals.h"
#include "constants.h"


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
		filteredVal = (*it).x * K[kSize];
		sumK += K[kSize];
		// forward loop for points ahead of sample point
		for (auto itf = std::next(it, 1); itf != unfiltPts.end() && std::distance(it, itf) <= kSize; ++itf) {
			sumK += K[kSize + std::distance(it, itf) ];
			filteredVal += (*itf).x * K[kSize + std::distance(it, itf)];
		}
		// reverse loop for points behind of sample point
		for (auto itr = it; itr != unfiltPts.begin() && std::distance(itr, it) < kSize; --itr) {
			sumK += K[kSize - std::distance(std::prev(itr,1), it) ];
			filteredVal += (*std::prev(itr, 1)).x * K[kSize - std::distance(std::prev(itr, 1), it)];
		}
		filtPts.push_back(cv::Point((int)(filteredVal / sumK), (*it).y));
	}
}