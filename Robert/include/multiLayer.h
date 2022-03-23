#pragma once
#include <vector>
#include "myTypes.h"
#include "raster.h"
#include <opencv2/core.hpp>
#include "MaterialModel.h"
#include "input.h"

#include <iterator> 
#include <algorithm>

#ifndef MULTILAYER_H
#define MULTILAYER_H

///////////////////////////////////////  MultiLayerScaffold  ///////////////////////////////////////
class MultiLayerScaffold
{
public:
	// default constructor
	MultiLayerScaffold() {}
	MultiLayerScaffold(TableInput input, Raster raster_);

	std::vector<std::vector<Path>> path;
	Raster raster;
	std::vector<Segment> segments;
	
	void leadout(double length);

	//TODO: set theta for entire path
	//void theta(double angle);
	//void theta(std::vector<double> angle);

private:
	void _interpPathPoints(std::vector<cv::Point2i> inPts, double wayptSpc, std::vector<cv::Point2i>& outPts);
	void _makePath(TableInput input);
	
};

inline MultiLayerScaffold::MultiLayerScaffold(TableInput input, Raster raster_)
{
	raster = raster_;
	_makePath(input);
}

inline void MultiLayerScaffold::leadout(double length)
{
	switch (segments.back().dir())
	{
	case printDir::X_POS: 
		segments.back().setScanDonePt(segments.back().scanDonePt() + cv::Point2d(length, 0));
		break;
	case printDir::X_NEG: 
		segments.back().setScanDonePt(segments.back().scanDonePt() - cv::Point2d(length, 0));
		break;
	case printDir::Y_POS:
		segments.back().setScanDonePt(segments.back().scanDonePt() + cv::Point2d(0, length));
		break;
	case printDir::Y_NEG:
		segments.back().setScanDonePt(segments.back().scanDonePt() - cv::Point2d(0, length));
		break;
	}
}

inline void MultiLayerScaffold::_interpPathPoints(std::vector<cv::Point2i> inPts, double wayptSpc, std::vector<cv::Point2i>& outPts)
{
	cv::Point2d diff, delta;
	double L;

	// Interpolate the points
	for (auto it = inPts.begin(); it != std::prev(inPts.end(), 1); ++it) {
		diff = *std::next(it, 1) - *it;
		L = cv::norm(diff);
		delta = (diff / L) * (double)MM2PIX(wayptSpc);
		for (int i = 0; cv::norm(i * delta) < L; i++) {
			outPts.push_back(*it + cv::Point2i(i * delta));
		}
	}
}

inline void MultiLayerScaffold::_makePath(TableInput input)
{
	cv::Rect roi;
	std::vector<cv::Point2i> wp_px;
	std::vector<cv::Point2d> wp_mm;
	cv::Point2d scanDonePt;
	int pixRodWth = MM2PIX(raster.rodWidth());
	int direction = 1;
	cv::Point2d origin = raster.origin();
	std::vector<Path> tmp;

	double z = input.initPos.z;
	double T = 0;
	double e = input.E;
	double f = input.F;
	double w = 0;

	// Rods
	for (int layer = input.startLayer; layer < input.layers + input.startLayer; layer++)
	{
		for (auto it = raster.px(layer).begin(); it != std::prev(raster.px(layer).end()); ++it) 
		{
			// Generate the pixel waypoints
			_interpPathPoints(std::vector<cv::Point2i> {*it, * std::next(it)}, input.wayptSpc, wp_px);

			// add the final point of the pattern
			if (std::next(it) == std::prev(raster.px(layer).end())) {
				wp_px.push_back(*std::next(it));
			}

			// making the path
			wp_mm.resize(wp_px.size());
			std::transform(wp_px.begin(), wp_px.end(), wp_mm.begin(), [&origin](cv::Point& pt) {return (PIX2MM(cv::Point2d(pt)) + origin); });
			for (auto it2 = wp_mm.begin(); it2 != wp_mm.end(); ++it2) {
				tmp.push_back(Path(*it2, z, T, f, e, w));
			}
			path.push_back(tmp);

			// Determine the direction 
			if (((*std::next(it)).x - (*it).x) > 0) { direction = printDir::X_POS; }	// positive x direction
			else if (((*std::next(it)).x - (*it).x) < 0) { direction = printDir::X_NEG; }	// negative x direction
			else if (((*std::next(it)).y - (*it).y) > 0) { direction = printDir::Y_POS; }	// positive y direction
			else if (((*std::next(it)).y - (*it).y) < 0) { direction = printDir::Y_NEG; }	// negative y direction

			// Setting the scan done point and the segment roi
			scanDonePt = wp_mm.front();
			// ----------- FIRST/THIRD LAYER -----------
			if (layer % 2 == 0) 
			{
				switch (direction % 2) {
				case 0: // Horizontal lines
					roi = cv::Rect(*it - cv::Point(0, pixRodWth / 2), *std::next(it, 1) + cv::Point(0, pixRodWth / 2));
					// shifting and stretching the roi
					roi -= cv::Point(pixRodWth / 4, 0);
					roi += cv::Size(pixRodWth / 2, 0);
					// defining the point when the region has been completely scanned as the end of the next horizontal line
					scanDonePt += (layer % 4 == 0) ? cv::Point2d(0, raster.spacing()) : cv::Point2d(0, -raster.spacing());
					// if it is the final segment
					if (std::next(it) == std::prev(raster.px(layer).end())) {
						scanDonePt = wp_mm.back();
					}
					break;
				case 1: // vertical lines
					roi = cv::Rect(0, 0, 1, 1);
					//roi = cv::Rect(*it - cv::Point(pixRodWth / 2, 0), *std::next(it, 1) + cv::Point(pixRodWth / 2, 0));
					// defining the point when the region has been completely scanned as the midpoint of the next vertical line

					// if it's not the last vertical rod
					if (std::next(it, 2) != std::prev(raster.px(layer).end())) {
						scanDonePt += (layer % 4 == 0) ? cv::Point2d(raster.length(), 1.5 * raster.spacing()) : cv::Point2d(raster.length(), -1.5 * raster.spacing());
					}
					else {
						scanDonePt += (layer % 4 == 0) ? cv::Point2d(raster.length(), raster.spacing()) : cv::Point2d(raster.length(), -raster.spacing());
					}
					if (!raster.roi(layer).contains(scanDonePt)) {
						scanDonePt -= cv::Point2d(raster.length() * 2, 0);
					}
					break;
				}
			}
			// ----------- SECOND/FOURTH LAYER -----------
			else if (layer % 2 == 1)
			{
				switch (direction % 2) {
				case 0: // Horizontal lines
					roi = cv::Rect(0, 0, 1, 1);
					
					// defining the point when the region has been completely scanned as the midpoint of the next horizontal line
					// if it's not the last horizontal rod
					if (std::next(it, 2) != std::prev(raster.px(layer).end())) {
						scanDonePt += (layer % 4 == 1) ? cv::Point2d(1.5 * raster.spacing(), raster.length()) : cv::Point2d(-1.5 * raster.spacing(), raster.length());
					}
					else {
						scanDonePt += (layer % 4 == 1) ? cv::Point2d(raster.spacing(), raster.length()) : cv::Point2d(-raster.spacing(), raster.length());
					}
					if (!raster.roi(layer).contains(scanDonePt)) {
						scanDonePt -= cv::Point2d(0, raster.length() * 2);
					}
					break;
				case 1: // vertical lines
					roi = cv::Rect(*it - cv::Point(pixRodWth / 2, 0), *std::next(it, 1) + cv::Point(pixRodWth / 2, 0));
					// // shifting and stretching the roi
					roi -= cv::Point(0, pixRodWth / 4);
					roi += cv::Size(0, pixRodWth / 2);
					// defining the point when the region has been completely scanned as the end of the next vertical line
					scanDonePt += (layer % 4 == 1) ? cv::Point2d(raster.spacing(), 0) : cv::Point2d(-raster.spacing(), 0);
					// if it is the final segment
					if (std::next(it) == std::prev(raster.px(layer).end())) {
						scanDonePt = wp_mm.back();
					}
					break;
				}
			}
			segments.push_back(Segment(roi, wp_px, scanDonePt, direction, layer));
			wp_px.clear();
			wp_mm.clear();
			tmp.clear();
		}
		z += input.height;
	}
}


///////////////////////////////////////  FunGenScaf  ///////////////////////////////////////

class FunGenScaf : public MultiLayerScaffold
{
public:
	FunGenScaf() {};
	FunGenScaf(TableInput input, Raster raster_, MaterialModel matModel);

private:
	void _makeFGS(char scafType, double range[2]);
	void _setInput(MaterialModel matModel);


};

FunGenScaf::FunGenScaf(TableInput input, Raster raster_, MaterialModel matModel)
	: MultiLayerScaffold(input, raster_)
{
	_makeFGS(input.type, input.range);
	_setInput(matModel);
}

inline void FunGenScaf::_makeFGS(char scafType, double range[2])
{
	int numPts;
	double delta;
	double width = range[0];

	if (scafType != 'b' && scafType != 'g' && scafType != 'c') {
		std::cout << "ERROR: unknown scaffold type" << std::endl;
		return;
	}

	switch (scafType)
	{
	default:
		break;
	case 'b': // bowtie scaffold
	{
		numPts = path[0].size() - 1;
		delta = (range[1] - range[0]) / (numPts / 2.0);

		for (auto it_seg = path.begin(); it_seg != path.end(); ++it_seg) {
			for (auto it_rod = (*it_seg).begin(); it_rod != (*it_seg).end(); ++it_rod) {

				// Modify the width
				(*it_rod).w = width;
				// if long rods
				if (std::distance(path.begin(), it_seg) % 2 == 0) {
					// decrease for first half of rod then increase for second half
					if (std::distance((*it_seg).begin(), it_rod) < (numPts / 2.0)) {
						width += delta;
					}
					else {
						width -= delta;
					}
				}
				else {
					width = range[0];
				}
			}
		}
		break;
	}
	case 'g': // gradient scaffold
	{
		numPts = path[0].size() - 1;
		delta = (range[1] - range[0]) / numPts;

		for (auto it_seg = path.begin(); it_seg != path.end(); ++it_seg) {
			for (auto it_rod = (*it_seg).begin(); it_rod != (*it_seg).end(); ++it_rod) {

				// Modify the width
				(*it_rod).w = width;
				switch (std::distance(path.begin(), it_seg) % 4)
				{
				case 0: // positive x direction rods
					width += delta;
					break;
				case 1: // y direction rods at max x
					width = range[1];
					break;
				case 2: // negative x direction rods
					width -= delta;
					break;
				case 3: // y direction rods at min x
					width = range[0];
					break;
				}
			}
		}
		break;
	}
	case 'c': // continuous gradient scaffold
	{
		numPts = ceil((path[0].size() - 1) * ceil(path.size() / 2.0));
		delta = (range[1] - range[0]) / numPts;

		for (auto it_seg = path.begin(); it_seg != path.end(); ++it_seg) {
			for (auto it_rod = (*it_seg).begin(); it_rod != (*it_seg).end(); ++it_rod) {

				// Modify the width
				(*it_rod).w = width;
				// if long rods
				if (std::distance(path.begin(), it_seg) % 2 == 0) {
					width += delta;
				}
			}
		}
		break;
	}
	}
}

inline void FunGenScaf::_setInput(MaterialModel matModel)
{
	// modify the inputs
	for (auto it_seg = path.begin(); it_seg != path.end(); ++it_seg) {
		for (auto it_rod = (*it_seg).begin(); it_rod != (*it_seg).end(); ++it_rod) {

			// Modify the width
			switch (matModel.type())
			{
			case 'f':
				(*it_rod).f = matModel.output((*it_rod).w, (*it_rod).e);
				break;
			case 'a':
				(*it_rod).e = matModel.output((*it_rod).w, (*it_rod).f);
				break;
			}
		}
	}
}


#endif // !MULTILAYER_H

