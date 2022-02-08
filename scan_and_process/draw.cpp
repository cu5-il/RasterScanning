#include <iostream>
#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iterator>

void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == cv::EVENT_LBUTTONDOWN) {
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
}

static cv::Scalar randomColor(cv::RNG& rng){
	int icolor = (unsigned)rng;
	return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}

void addScale(cv::Mat& image, double length = 1, cv::Point offset = cv::Point(25, 25), double fontScale = 0.5) {
	cv::Point location(offset.x, image.rows - offset.y);
	char buff[50];
	snprintf(buff, sizeof(buff), "%gmm", length);
	cv::putText(image, buff, location, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(255, 255, 255), 1, cv::LINE_8);
	cv::rectangle(image, cv::Rect(location.x + 6, location.y + 5, MM2PIX(length), 5), cv::Scalar(255, 255, 255), -1);
}

void drawEdges(cv::Mat src, cv::Mat&  dst, cv::Mat edges, const cv::Scalar& color, const int pointSz = 1) {
	
	// copy the source to the destination
	src.copyTo(dst);
	if (dst.channels() < 3) {
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
	}

	// Copy the global edges to the  image
	if (pointSz > 0) {
		// show the edges as filled circles
		std::vector<cv::Point> edgePts;
		cv::findNonZero(edges, edgePts);
		for (auto it = edgePts.begin(); it != edgePts.end(); ++it) {
			cv::circle(dst, *it, pointSz, color, -1, cv::LINE_AA);
		}
	}
	else {
		// show the edges as pixels
		cv::Mat(edges.size(), CV_8UC3, color).copyTo(dst, edges);
	}
}

void drawErrors(cv::Mat src, cv::Mat& dst, std::vector<Segment>& seg) {
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

void drawSegments(cv::Mat src, cv::Mat& dst, std::vector<Segment>& seg, const int pointSz = 1) {
	cv::Point origin = seg[0].waypoints()[0];
	cv::Point ScanDonePt(0,0);
	cv::Scalar color = cv::Scalar(255, 255, 0);
	cv::RNG rng(0xFFFFFFFF);
	int segNum = 0;
	int samePtCnt = 0;
	char buff[50];
	cv::Size textSz;
	int baseline = 0;

	// copy the source to the destination
	src.copyTo(dst);
	if (dst.channels() < 3) {
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
	}

	// getting the values from each segment
	for (auto it = seg.begin(); it != seg.end(); ++it, segNum++) {
		// Randomize the color
		color = randomColor(rng);
		// Draw the segment boundaries
		cv::rectangle(dst, (*it).ROI(), color);
		// Draw and label the scan done point
		snprintf(buff, sizeof(buff), "%u", segNum);
		ScanDonePt = origin + cv::Point(MM2PIX((*it).scanDonePt().x), MM2PIX((*it).scanDonePt().y));
		if (it != seg.begin() && ScanDonePt == origin + cv::Point(MM2PIX((*std::prev(it)).scanDonePt().x), MM2PIX((*std::prev(it)).scanDonePt().y))) {
			samePtCnt++;
			textSz = cv::getTextSize(buff, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
			cv::putText(dst, buff, ScanDonePt + cv::Point(pointSz + samePtCnt * textSz.width, -2), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_8);
		}
		else {
			cv::putText(dst, buff, ScanDonePt + cv::Point(pointSz, -2), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_8);
			samePtCnt = 0;
		}
		cv::circle(dst, ScanDonePt, pointSz, color, -1, cv::LINE_AA);
	}
}